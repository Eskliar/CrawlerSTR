from django.shortcuts import render
from django.http import HttpResponse, JsonResponse
from django.views.decorators.csrf import csrf_exempt
import json
from . models import datosCrawler
from django.core.cache import cache
# Create your views here.
#Lo que se envía al cliente

matriz = [[0 for _ in range(9)] for _ in range(9)]  # Matriz 9x9 llena de ceros

def mostrar_matriz(request):
    matriz = cache.get('matriz_global', [[0 for _ in range(9)] for _ in range(9)])  # Por defecto, una matriz 9x9 de ceros
    return render(request, "matriz.html", {"matriz": matriz})

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
