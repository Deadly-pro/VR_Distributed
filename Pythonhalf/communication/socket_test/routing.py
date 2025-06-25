from django.urls import re_path
from . import consumers

websocket_urlpatterns = [
    re_path(r'ws/stream/$', consumers.StreamingConsumer.as_asgi()),
]
