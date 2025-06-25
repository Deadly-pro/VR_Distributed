"""
URL configuration for django_test project.

The `urlpatterns` list routes URLs to views. For more information please see:
    https://docs.djangoproject.com/en/5.2/topics/http/urls/
Examples:
Function views
    1. Add an import:  from my_app import views
    2. Add a URL to urlpatterns:  path('', views.home, name='home')
Class-based views
    1. Add an import:  from other_app.views import Home
    2. Add a URL to urlpatterns:  path('', Home.as_view(), name='home')
Including another URLconf
    1. Import the include() function: from django.urls import include, path
    2. Add a URL to urlpatterns:  path('blog/', include('blog.urls'))
"""

from django.contrib import admin
from django.urls import path, re_path
from django.urls import include
from test_app.views import index
from test_app.views import media_image
from django.conf import settings
from django.conf.urls.static import static
from stream.views import mediapipe, camera_feed

urlpatterns = [
    path("admin/", admin.site.urls),
    path("", index),
    path("test/", include("test_app.urls")),
    path('video_test/', include('socket_test.urls', namespace='socket_test')),
    path('stream/', include('stream.urls')),
    path('mediapipe/', mediapipe, name='mediapipe'),
    path('camera_feed/', camera_feed, name='camera_feed'),
    re_path(r'^(?P<filename>[\w\-]+\.(jpg|png|gif|ico|mp4))$',media_image, name='media_image'),
]
