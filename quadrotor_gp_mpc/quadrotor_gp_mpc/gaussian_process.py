#!/usr/bin/env python3
"""
Gaussian Process for Model Learning

This module implements a Gaussian Process for learning model uncertainties
in the quadrotor dynamics. It can learn aerodynamic effects, parameter
variations, and other model discrepancies.
"""

import numpy as np
from typing import Tuple, List, Optional
from scipy.optimize import minimize
from scipy.linalg import cholesky, cho_solve, LinAlgError
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class RBFKernel:
    """Radial Basis Function (RBF) kernel for Gaussian Process."""

    def __init__(self, length_scale: float = 1.0, signal_variance: float = 1.0):
        self.length_scale = length_scale
        self.signal_variance = signal_variance

    def __call__(self, X1: np.ndarray, X2: np.ndarray) -> np.ndarray:
        """
        Compute RBF kernel matrix.

        Args:
            X1: Input matrix (n1 x d)
            X2: Input matrix (n2 x d)

        Returns:
            Kernel matrix (n1 x n2)
        """
        # Compute squared Euclidean distances
        dists = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
        dists = np.maximum(dists, 0)  # Numerical stability

        return self.signal_variance * np.exp(-0.5 * dists / self.length_scale**2)

    def gradient(self, X1: np.ndarray, X2: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute gradients of kernel w.r.t. hyperparameters.

        Returns:
            Tuple of gradients w.r.t. (length_scale, signal_variance)
        """
        K = self(X1, X2)
        dists = np.sum(X1**2, 1).reshape(-1, 1) + np.sum(X2**2, 1) - 2 * np.dot(X1, X2.T)
        dists = np.maximum(dists, 0)

        # Gradient w.r.t. length_scale
        dK_dl = K * dists / self.length_scale**3

        # Gradient w.r.t. signal_variance
        dK_ds = K / self.signal_variance

        return dK_dl, dK_ds


class GaussianProcess(Node):
    """
    Gaussian Process for learning model uncertainties.

    This GP learns the discrepancy between the nominal model and
    the true dynamics: f_true = f_nominal + f_gp + noise
    """

    def __init__(self, input_dim: int = 16, output_dim: int = 12):
        super().__init__('gaussian_process')

        self.input_dim = input_dim    # [state, control] = [12, 4]
        self.output_dim = output_dim  # state derivative dim = 12

        # GP hyperparameters
        self.kernel = RBFKernel(length_scale=1.0, signal_variance=1.0)
        self.noise_variance = 0.01

        # Training data
        self.X_train = np.empty((0, self.input_dim))
        self.Y_train = np.empty((0, self.output_dim))

        # For efficient inference
        self.K_inv = None
        self.L = None  # Cholesky decomposition
        self.alpha = None  # K^{-1} y

        # Data collection parameters
        self.max_data_points = 1000
        self.data_collection_active = True

        # ROS interfaces
        self.prediction_pub = self.create_publisher(
            Float64MultiArray,
            '/gp/prediction',
            10
        )

        self.uncertainty_pub = self.create_publisher(
            Float64MultiArray,
            '/gp/uncertainty',
            10
        )

        self.training_data_sub = self.create_subscription(
            Float64MultiArray,
            '/gp/training_data',
            self.training_data_callback,
            10
        )

        self.prediction_request_sub = self.create_subscription(
            Float64MultiArray,
            '/gp/prediction_request',
            self.prediction_request_callback,
            10
        )

        # Timer for periodic training
        self.training_timer = self.create_timer(5.0, self.train_gp)

        self.get_logger().info(f"Gaussian Process initialized: input_dim={input_dim}, output_dim={output_dim}")

    def add_training_data(self, X: np.ndarray, Y: np.ndarray):
        """
        Add new training data point(s).

        Args:
            X: Input data (n x input_dim)
            Y: Output data (n x output_dim)
        """
        X = np.atleast_2d(X)
        Y = np.atleast_2d(Y)

        if X.shape[1] != self.input_dim or Y.shape[1] != self.output_dim:
            self.get_logger().error(f"Data dimension mismatch: X={X.shape}, Y={Y.shape}")
            return

        # Add to training set
        self.X_train = np.vstack([self.X_train, X])
        self.Y_train = np.vstack([self.Y_train, Y])

        # Remove oldest data if we exceed max capacity
        if len(self.X_train) > self.max_data_points:
            excess = len(self.X_train) - self.max_data_points
            self.X_train = self.X_train[excess:]
            self.Y_train = self.Y_train[excess:]

        # Invalidate cached computations
        self.K_inv = None
        self.L = None
        self.alpha = None

        self.get_logger().debug(f"Added training data. Total points: {len(self.X_train)}")

    def compute_kernel_matrix(self, X1: np.ndarray, X2: Optional[np.ndarray] = None) -> np.ndarray:
        """Compute kernel matrix with noise for training data."""
        if X2 is None:
            X2 = X1
            add_noise = True
        else:
            add_noise = False

        K = self.kernel(X1, X2)

        if add_noise and X1 is X2:
            K += self.noise_variance * np.eye(len(X1))

        return K

    def fit(self):
        """Fit the GP to current training data."""
        if len(self.X_train) < 2:
            self.get_logger().warning("Insufficient training data for GP fitting")
            return

        try:
            # Compute kernel matrix
            K = self.compute_kernel_matrix(self.X_train)

            # Cholesky decomposition for numerical stability
            self.L = cholesky(K, lower=True)

            # Precompute alpha = K^{-1} y for each output dimension
            self.alpha = np.zeros((len(self.X_train), self.output_dim))
            for i in range(self.output_dim):
                self.alpha[:, i] = cho_solve((self.L, True), self.Y_train[:, i])

            self.get_logger().info(f"GP fitted with {len(self.X_train)} training points")

        except LinAlgError as e:
            self.get_logger().error(f"Numerical error in GP fitting: {e}")
            # Add jitter for numerical stability and reset alpha
            self.noise_variance *= 10
            if self.noise_variance > 1.0:
                self.noise_variance = 0.01
            # Initialize alpha to prevent NoneType errors
            self.alpha = np.zeros((len(self.X_train), self.output_dim))
            self.L = None

    def predict(self, X_test: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Make predictions at test points.

        Args:
            X_test: Test input points (n x input_dim)

        Returns:
            Tuple of (mean predictions, variance predictions)
        """
        X_test = np.atleast_2d(X_test)

        if len(self.X_train) == 0 or self.alpha is None:
            # No training data or not fitted
            mean = np.zeros((len(X_test), self.output_dim))
            var = np.ones((len(X_test), self.output_dim)) * self.kernel.signal_variance
            return mean, var

        try:
            # Compute kernel between test and training points
            K_star = self.kernel(self.X_train, X_test)

            # Mean prediction: K_star^T alpha
            mean = K_star.T @ self.alpha

            # Variance prediction
            K_star_star = self.kernel(X_test, X_test)
            v = cho_solve((self.L, True), K_star)
            var = np.diag(K_star_star) - np.sum(K_star * v, axis=0)
            var = np.maximum(var, 1e-10)  # Ensure positive variance
            var = np.tile(var.reshape(-1, 1), (1, self.output_dim))

            return mean, var

        except Exception as e:
            self.get_logger().error(f"Error in GP prediction: {e}")
            mean = np.zeros((len(X_test), self.output_dim))
            var = np.ones((len(X_test), self.output_dim)) * self.kernel.signal_variance
            return mean, var

    def log_marginal_likelihood(self) -> float:
        """Compute log marginal likelihood for hyperparameter optimization."""
        if len(self.X_train) < 2 or self.L is None:
            return -np.inf

        try:
            # Log determinant term: 2 * sum(log(diag(L)))
            log_det = 2 * np.sum(np.log(np.diag(self.L)))

            # Quadratic term: y^T K^{-1} y
            quad_term = 0
            for i in range(self.output_dim):
                quad_term += np.dot(self.Y_train[:, i], self.alpha[:, i])

            # Constant term
            n = len(self.X_train)
            const = n * self.output_dim * np.log(2 * np.pi)

            return -0.5 * (log_det + quad_term + const)

        except Exception as e:
            self.get_logger().error(f"Error computing log marginal likelihood: {e}")
            return -np.inf

    def optimize_hyperparameters(self):
        """Optimize hyperparameters by maximizing marginal likelihood."""
        if len(self.X_train) < 10:  # Need sufficient data
            return

        def objective(params):
            """Negative log marginal likelihood."""
            old_length_scale = self.kernel.length_scale
            old_signal_variance = self.kernel.signal_variance
            old_noise_variance = self.noise_variance

            # Update hyperparameters
            self.kernel.length_scale = np.exp(params[0])  # Log-space
            self.kernel.signal_variance = np.exp(params[1])
            self.noise_variance = np.exp(params[2])

            # Refit with new hyperparameters
            self.fit()

            # Compute negative log marginal likelihood
            nll = -self.log_marginal_likelihood()

            # Restore if numerical issues
            if not np.isfinite(nll):
                self.kernel.length_scale = old_length_scale
                self.kernel.signal_variance = old_signal_variance
                self.noise_variance = old_noise_variance
                nll = 1e6

            return nll

        # Initial parameters (log-space)
        x0 = np.array([
            np.log(self.kernel.length_scale),
            np.log(self.kernel.signal_variance),
            np.log(self.noise_variance)
        ])

        try:
            result = minimize(objective, x0, method='L-BFGS-B',
                            options={'maxiter': 50})

            if result.success:
                # Update with optimized parameters
                self.kernel.length_scale = np.exp(result.x[0])
                self.kernel.signal_variance = np.exp(result.x[1])
                self.noise_variance = np.exp(result.x[2])
                self.fit()  # Refit with optimized hyperparameters

                self.get_logger().info(
                    f"Hyperparameters optimized: "
                    f"length_scale={self.kernel.length_scale:.3f}, "
                    f"signal_variance={self.kernel.signal_variance:.3f}, "
                    f"noise_variance={self.noise_variance:.3f}"
                )

        except Exception as e:
            self.get_logger().error(f"Hyperparameter optimization failed: {e}")

    def training_data_callback(self, msg: Float64MultiArray):
        """Callback for receiving training data."""
        data = np.array(msg.data)

        expected_size = self.input_dim + self.output_dim
        if len(data) != expected_size:
            self.get_logger().error(f"Invalid training data size: {len(data)} != {expected_size}")
            return

        X = data[:self.input_dim].reshape(1, -1)
        Y = data[self.input_dim:].reshape(1, -1)

        self.add_training_data(X, Y)

    def prediction_request_callback(self, msg: Float64MultiArray):
        """Callback for prediction requests."""
        X_test = np.array(msg.data).reshape(1, -1)

        if X_test.shape[1] != self.input_dim:
            self.get_logger().error(f"Invalid prediction input size: {X_test.shape[1]} != {self.input_dim}")
            return

        mean, var = self.predict(X_test)

        # Publish prediction
        pred_msg = Float64MultiArray()
        pred_msg.data = mean.flatten().tolist()
        self.prediction_pub.publish(pred_msg)

        # Publish uncertainty
        unc_msg = Float64MultiArray()
        unc_msg.data = np.sqrt(var).flatten().tolist()
        self.uncertainty_pub.publish(unc_msg)

    def train_gp(self):
        """Periodic training callback."""
        if len(self.X_train) >= 10:
            # Optimize hyperparameters occasionally
            if len(self.X_train) % 50 == 0:
                self.optimize_hyperparameters()
            else:
                self.fit()

    def save_model(self, filename: str):
        """Save trained GP model."""
        np.savez(filename,
                X_train=self.X_train,
                Y_train=self.Y_train,
                length_scale=self.kernel.length_scale,
                signal_variance=self.kernel.signal_variance,
                noise_variance=self.noise_variance)

        self.get_logger().info(f"GP model saved to {filename}")

    def load_model(self, filename: str):
        """Load trained GP model."""
        try:
            data = np.load(filename)
            self.X_train = data['X_train']
            self.Y_train = data['Y_train']
            self.kernel.length_scale = float(data['length_scale'])
            self.kernel.signal_variance = float(data['signal_variance'])
            self.noise_variance = float(data['noise_variance'])

            self.fit()
            self.get_logger().info(f"GP model loaded from {filename}")

        except Exception as e:
            self.get_logger().error(f"Failed to load GP model: {e}")


def main(args=None):
    rclpy.init(args=args)

    gp_node = GaussianProcess()

    try:
        rclpy.spin(gp_node)
    except KeyboardInterrupt:
        pass
    finally:
        gp_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
