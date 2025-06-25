from django.urls import path
from .views import index as socket_test_index

app_name = "socket_test"  # <-- Add this line

urlpatterns = [
    path("", socket_test_index, name='stream'),  # Change name to 'stream'
]
