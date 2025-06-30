"""
ASGI config for django_test project.

It exposes the ASGI callable as a module-level variable named ``application``.

For more information on this file, see
https://docs.djangoproject.com/en/5.2/howto/deployment/asgi/
"""

import os

from django.core.asgi import get_asgi_application
from channels.routing import ProtocolTypeRouter, URLRouter
from channels.auth import AuthMiddlewareStack
import socket_test.routing# Import your chat routing module if you have one

os.environ.setdefault("DJANGO_SETTINGS_MODULE", "django_test.settings")


application = ProtocolTypeRouter({
    "http": get_asgi_application(),
    "websocket": AuthMiddlewareStack(
        URLRouter(
            socket_test.routing.websocket_urlpatterns,  
            
        )  # Provide an empty list if you have no routes yet
    ),
    # Add other protocol handlers as needed
})
