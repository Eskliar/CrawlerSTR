from django.urls import path
from . import views #Desde la carpeta actual '.' importar views

urlpatterns=[
    path('', views.mostrar_matriz, name='mostrar_matriz'),#Ruta principal, localhost:8000
    path('about/', views.about),
    path('api/recibir_dato/', views.recibir_dato, name='recibir_dato'),
    path('start/', views.start_button, name='start_button'),
    path('stop/', views.stop_button, name='stop_button'),
    path('get_start_state/', views.get_start_state, name='get_start_state'),
    path('api/recibir_estado/',views.recibir_estado, name='recibir_estado'),
    path('api/get_estado_crawler/',views.get_estado_crawler, name='get_estado_crawler'),
    path('api/get_matriz/',views.get_matriz,name="get_matriz"),
    path('api/get_direccion_crawler/', views.get_direccion_crawler, name='get_direccion_crawler'),
    path('api/set_direccion_crawler/', views.set_direccion_crawler, name='set_direccion_crawler'),

]
    
