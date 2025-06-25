from django.shortcuts import render
from django.http import FileResponse, Http404
import os
from django.conf import settings

# Create your views here.
def index(request, *args, **kwargs):
   return render(request, 'index.html')

def test(request, *args, **kwargs):
   return render(request, 'test_app/text.html')

def test2(request, *args, **kwargs):
   return render(request, 'test.html')

def media_image(request, filename):
    """
    Serve media images from the 'media' directory.
    """
    media_path = os.path.join(settings.MEDIA_ROOT, filename)
    if not os.path.exists(media_path):
        raise Http404("File not found")
    return FileResponse(open(media_path, 'rb'))