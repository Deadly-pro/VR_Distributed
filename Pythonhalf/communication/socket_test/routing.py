from django.urls import re_path
from . import consumers

websocket_urlpatterns = [
    re_path(r'ws/webrtc/(?P<room_name>\w+)/$', consumers.WebRTCStreamingConsumer.as_asgi()),
]