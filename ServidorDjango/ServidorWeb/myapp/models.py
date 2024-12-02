from django.db import models

# Create your models here.
# Tablas para que el ORM maneje
class datosCrawler(models.Model):
    dato = models.CharField(max_length=200000)
    valor = models.DecimalField(decimal_places=5, max_digits=10)
    timestamp = models.DateTimeField(auto_now_add=True)
    
#    def __init__(self, dato,valor):
 #       self.dato=dato
  #      self.valor=valor