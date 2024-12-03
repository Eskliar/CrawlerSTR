from django.shortcuts import render
from django.http import HttpResponse, JsonResponse
from django.views.decorators.csrf import csrf_exempt
import json
from . models import datosCrawler
from django.core.cache import cache
# Create your views here.
#Lo que se envía al cliente


# Variable global para el estado
start = False  # Inicialmente apagado
aprendiendoEjecutando=0 #-1-Detenido, 0-Aprendiendo, 1-Ejecutando
textoEstadoCrawler="Detenid" #Detenido, Aprendiendo, Ejecutando lo aprendido


matriz = [[0 for _ in range(9)] for _ in range(9)]  # Matriz 9x9 llena de ceros
cache.set('matriz_global', matriz)

def mostrar_matriz(request):
    matriz = cache.get('matriz_global', [[0 for _ in range(9)] for _ in range(9)])  # Por defecto, una matriz 9x9 de ceros
    estado_crawler = cache.get('estado_crawler', textoEstadoCrawler)
    return render(request, "matriz.html", {
        "matriz": matriz,
        "estado_crawler": estado_crawler
    })
    
def get_matriz(request):
    ##matriz = cache.get('matriz_global', matriz)  # Supongamos que la matriz está almacenada en caché
    matriz = cache.get('matriz_global', []) 
    return JsonResponse({'matriz': matriz})

def about(request):
    return HttpResponse("About")

@csrf_exempt
def recibir_dato(request):
    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            matriz = data.get('matriz')  # Intentamos extraer la matriz

            # Valida si la matriz es válida (9x9)
            if matriz and len(matriz) == 9 and all(len(row) == 9 for row in matriz):
                # Guarda la matriz en la caché
                cache.set('matriz_global', matriz, timeout=None)  # Sin límite de tiempo
                return JsonResponse({'status': 'success', 'mensaje': 'Matriz recibida y almacenada'}, status=200)
            else:
                return JsonResponse({'status': 'error', 'mensaje': 'La matriz debe ser 9x9'}, status=400)
        except json.JSONDecodeError:
            return JsonResponse({'status': 'error', 'mensaje': 'JSON inválido'}, status=400)
    else:
        return JsonResponse({'status': 'error', 'mensaje': 'Solo acepta POST requests'}, status=405)

# Vista para manejar el botón Start
@csrf_exempt
def start_button(request):
    global start
    if request.method == 'POST':
        start = True
        return JsonResponse({'status': 'success', 'message': 'Start activated', 'start': start})
    print(start)
    return JsonResponse({'status': 'error', 'message': 'Invalid request'}, status=405)

# Vista para manejar el botón Stop
@csrf_exempt
def stop_button(request):
    global start
    if request.method == 'POST':
        start = False
        return JsonResponse({'status': 'success', 'message': 'Stop activated', 'start': start})
    print(start)
    return JsonResponse({'status': 'error', 'message': 'Invalid request'}, status=405)

# Método GET para consultar el estado de "start"
def get_start_state(request):
    global start
    print("Boton start/stop:")
    print(start)
    response = JsonResponse({'start': start})
    print(response.content.decode('utf-8'))
    response['Content-Type'] = 'application/json'
    return response


@csrf_exempt
def recibir_estado(request):
    global textoEstadoCrawler
    if request.method == 'POST':
        try:
            data = json.loads(request.body)
            EstadoAprendiendoEjecutando = data.get('estado') 
            print("Dato recibido estado: ")
            print(EstadoAprendiendoEjecutando)# Intentamos extraer la matriz
            if(EstadoAprendiendoEjecutando==-1):
                textoEstadoCrawler="Detenido"
            elif(EstadoAprendiendoEjecutando==0):
                textoEstadoCrawler="Aprendiendo"
            elif(EstadoAprendiendoEjecutando==1):
                textoEstadoCrawler="Ejecutando lo aprendido"
            cache.set('estado_crawler', textoEstadoCrawler, timeout=None)
            print("Estado crawler:")
            print(textoEstadoCrawler)# Sin límite de tiempo
            cache.set('estado_crawler', textoEstadoCrawler, timeout=None)
            return JsonResponse({'status': 'success', 'mensaje': 'Estado recibido y almacenado'}, status=200)
        except json.JSONDecodeError:
            return JsonResponse({'status': 'error', 'mensaje': 'JSON inválido'}, status=400)
    else:
        return JsonResponse({'status': 'error', 'mensaje': 'Solo acepta POST requests'}, status=405)

def get_estado_crawler(request):
    estado_crawler = cache.get('estado_crawler', textoEstadoCrawler)
    return JsonResponse({"estado_crawler": estado_crawler})