from django.urls import path
from .views import camera_feed

urlpatterns = [
    path('', camera_feed, name='video_page'),
]
