"""
Input validation and sanitization middleware
"""
import logging
import html
import re
from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse

logger = logging.getLogger(__name__)

class InputSanitizer:
    """
    Utility class for sanitizing user inputs
    """

    @staticmethod
    def sanitize_text(text: str) -> str:
        """
        Sanitize text input by removing potentially harmful content
        """
        if not text:
            return text

        # Remove potentially harmful HTML tags
        sanitized = html.escape(text)

        # Remove any script tags (case insensitive)
        sanitized = re.sub(r'<script[^>]*>.*?</script>', '', sanitized, flags=re.IGNORECASE | re.DOTALL)

        # Remove javascript: and other potentially harmful protocols
        sanitized = re.sub(r'javascript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'vbscript:', '', sanitized, flags=re.IGNORECASE)
        sanitized = re.sub(r'on\w+\s*=', '', sanitized, flags=re.IGNORECASE)

        # Remove potentially harmful characters at the beginning and end
        sanitized = sanitized.strip()

        return sanitized

    @staticmethod
    def validate_query_content(content: str) -> bool:
        """
        Validate query content for potentially malicious patterns
        """
        if not content:
            return False

        # Check for SQL injection patterns
        sql_patterns = [
            r"(\b(SELECT|INSERT|UPDATE|DELETE|DROP|CREATE|ALTER|EXEC|UNION)\b)",
            r"(--|#|/\*|\*/)",
            r"(\b(OR|AND)\s+1\s*=\s*1\b)"
        ]

        for pattern in sql_patterns:
            if re.search(pattern, content, re.IGNORECASE):
                return False

        # Check for XSS patterns
        xss_patterns = [
            r"<script",
            r"javascript:",
            r"on\w+\s*=",
            r"expression\s*\("
        ]

        for pattern in xss_patterns:
            if re.search(pattern, content, re.IGNORECASE):
                return False

        return True

async def input_validation_middleware(request: Request, call_next):
    """
    Middleware to validate and sanitize inputs
    """
    # For POST requests, we need to read the body
    if request.method in ["POST", "PUT", "PATCH"]:
        try:
            body = await request.body()
            if body:
                # For now, just log the request - in a real implementation
                # you'd want to validate the specific fields based on the endpoint
                pass
        except Exception as e:
            logger.error(f"Error reading request body: {e}")

    response = await call_next(request)
    return response

def validate_and_sanitize_user_query(query_content: str) -> str:
    """
    Validate and sanitize a user query
    """
    if not query_content:
        raise HTTPException(status_code=400, detail="Query content cannot be empty")

    # Validate the content
    if not InputSanitizer.validate_query_content(query_content):
        raise HTTPException(status_code=400, detail="Query contains invalid content")

    # Sanitize the content
    sanitized_content = InputSanitizer.sanitize_text(query_content)

    # Additional check: ensure content is not too long
    if len(sanitized_content) > 1000:
        raise HTTPException(status_code=400, detail="Query too long, maximum 1000 characters")

    return sanitized_content