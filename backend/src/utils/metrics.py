"""
Performance monitoring and metrics utilities
"""
import time
import logging
from functools import wraps
from typing import Callable, Any
import statistics

logger = logging.getLogger(__name__)

class MetricsCollector:
    """
    Utility class for collecting and reporting performance metrics
    """
    def __init__(self):
        self.response_times = []
        self.request_counts = {
            'chat': 0,
            'embeddings': 0,
            'health': 0
        }
        self.error_counts = {
            'chat': 0,
            'embeddings': 0,
            'health': 0
        }

    def record_response_time(self, endpoint: str, response_time: float):
        """
        Record response time for an endpoint
        """
        self.response_times.append({
            'endpoint': endpoint,
            'response_time': response_time,
            'timestamp': time.time()
        })

        # Keep only the last 1000 measurements to prevent memory issues
        if len(self.response_times) > 1000:
            self.response_times = self.response_times[-1000:]

    def record_request(self, endpoint: str):
        """
        Record a request to an endpoint
        """
        if endpoint in self.request_counts:
            self.request_counts[endpoint] += 1
        else:
            self.request_counts[endpoint] = 1

    def record_error(self, endpoint: str):
        """
        Record an error for an endpoint
        """
        if endpoint in self.error_counts:
            self.error_counts[endpoint] += 1
        else:
            self.error_counts[endpoint] = 1

    def get_p95_response_time(self, endpoint: str = None) -> float:
        """
        Get the 95th percentile response time
        """
        if endpoint:
            times = [item['response_time'] for item in self.response_times
                    if item['endpoint'] == endpoint]
        else:
            times = [item['response_time'] for item in self.response_times]

        if not times:
            return 0.0

        # Calculate 95th percentile
        sorted_times = sorted(times)
        index = int(len(sorted_times) * 0.95)
        return sorted_times[min(index, len(sorted_times) - 1)] if sorted_times else 0.0

    def get_average_response_time(self, endpoint: str = None) -> float:
        """
        Get the average response time
        """
        if endpoint:
            times = [item['response_time'] for item in self.response_times
                    if item['endpoint'] == endpoint]
        else:
            times = [item['response_time'] for item in self.response_times]

        if not times:
            return 0.0

        return sum(times) / len(times)

    def get_request_rate(self, endpoint: str = None) -> float:
        """
        Get the request rate per minute
        """
        if endpoint:
            return self.request_counts.get(endpoint, 0)
        else:
            return sum(self.request_counts.values())

    def get_error_rate(self, endpoint: str = None) -> float:
        """
        Get the error rate
        """
        if endpoint:
            total_requests = self.request_counts.get(endpoint, 0)
            errors = self.error_counts.get(endpoint, 0)
        else:
            total_requests = sum(self.request_counts.values())
            errors = sum(self.error_counts.values())

        if total_requests == 0:
            return 0.0

        return (errors / total_requests) * 100

    def get_metrics_summary(self) -> dict:
        """
        Get a summary of all metrics
        """
        return {
            'response_times': {
                'p95': self.get_p95_response_time(),
                'avg': self.get_average_response_time(),
                'total_recorded': len(self.response_times)
            },
            'request_counts': self.request_counts.copy(),
            'error_counts': self.error_counts.copy(),
            'error_rates': {
                'overall': self.get_error_rate()
            }
        }

# Global metrics collector instance
metrics_collector = MetricsCollector()

def time_it(endpoint_name: str):
    """
    Decorator to time function execution and record metrics
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        async def wrapper(*args, **kwargs) -> Any:
            start_time = time.time()
            metrics_collector.record_request(endpoint_name)

            try:
                result = await func(*args, **kwargs)
                return result
            except Exception as e:
                metrics_collector.record_error(endpoint_name)
                raise e
            finally:
                response_time = time.time() - start_time
                metrics_collector.record_response_time(endpoint_name, response_time)

                # Log slow requests (those taking more than 3 seconds)
                if response_time > 3.0:
                    logger.warning(f"Slow request to {endpoint_name}: {response_time:.2f}s")
        return wrapper
    return decorator

def get_metrics_collector():
    """
    Get the global metrics collector instance
    """
    return metrics_collector