from django.urls import path
from . import views #Desde la carpeta actual '.' importar views

urlpatterns=[
    path('', views.mostrar_matriz, name='mostrar_matriz'),#Ruta principal, localhost:8000
    path('about/', views.about),
    path('api/recibir_dato/', views.recibir_dato, name='recibir_dato')
]
    
