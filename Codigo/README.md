# Código del Robot Auto-Balanceado R2-D2

Este directorio contiene el código utilizado para controlar el robot auto-balanceado inspirado en R2-D2. El sistema está programado en Arduino UNO e integra sensores, control PID, motores paso a paso y un módulo de seguridad para evitar caídas durante las pruebas.

## 1. Descripción general

El código implementa:
- Lectura del sensor MPU6050 (GY-521)
- Filtro complementario para estimación de ángulo
- Control PID (funcionando como PD agresivo)
- Control de dos motores NEMA17 mediante drivers step/dir
- Ajuste dinámico de potencia (perilla o comandos por Serial)
- Corte de seguridad si el ángulo es demasiado grande
- Trazas para el Serial Plotter

El objetivo del algoritmo es mantener al robot en posición vertical (0°), aplicando correcciones rápidas según la inclinación detectada.

## 2. Sensores (MPU6050)

El sistema realiza primero una calibración del giroscopio tomando 1000 muestras para eliminar el offset del eje Y.  
El ángulo del robot ("pitch") se calcula combinando:
- la inclinación del acelerómetro  
- la integración del giroscopio  
- un filtro complementario con α = 0.98  

Esto produce una señal estable y con baja deriva, ideal para el controlador PD.

## 3. Control de Motores

Se utilizan dos motores NEMA17 conectados a:
- HW-135 (motor izquierdo)
- EasyDriver (motor derecho)

El control se realiza mediante AccelStepper. Las funciones principales son:
- `aplicarLimitesMotores()` → ajusta velocidad y aceleración según la potencia real
- `mandarMotores(u)` → aplica la salida del PID (u) directamente a los motores
- `pararMotores()` → detiene el sistema si se detecta una condición de fallo

Para generar el torque que estabiliza el robot:
- motor izquierdo recibe `u`
- motor derecho recibe `-u`

## 4. Control PID (PD agresivo)

El PID opera con:
- Kp = 250  
- Ki = 0  
- Kd = 30  

El setpoint es 0°, lo que representa al robot vertical.  
Dado que Ki = 0, el controlador funciona como PD, evitando oscilaciones fuertes en pruebas iniciales y permitiendo una respuesta rápida.

## 5. Sistema de Seguridad

El sistema incluye protección basada en el ángulo:
- Si el robot supera ±30°, se detienen los motores (fallo latched)
- El sistema no se restablece automáticamente hasta que vuelva a ±10°
- Se envía un mensaje por Serial indicando el fallo y la recuperación

Esto evita daños en el robot durante las pruebas.

## 6. Control Dinámico de Potencia

El nivel de potencia se puede modificar mediante:
- un potenciómetro conectado al pin A0  
- comandos enviados por Serial (ejemplo: P=70 para 70%)

La potencia real nunca baja de 30% para evitar falta de reacción, y nunca sube de 100%.  
La potencia modifica:
- velocidad máxima
- aceleración
- respuesta del PID

## 7. Trazas (Serial Plotter)

La función `trazarSerial()` envía datos cada 20 ms al Serial Plotter:
- ángulo actual
- setpoint
- salida del PID
- nivel de potencia
- velocidad máxima efectiva
- aceleración efectiva

Esto permite depurar el comportamiento del robot en tiempo real.

## 8. Flujo principal del programa

### setup():
- Inicializa comunicación Serial
- Inicia MPU6050 y realiza la calibración del giroscopio
- Configura motores (velocidad/aceleración)
- Configura PID (límites, setpoint, modo automático)
- Inicia módulo de potencia
- Muestra instrucciones en el monitor Serial

### loop():
1. Actualiza la potencia mediante perilla o Serial  
2. Obtiene el ángulo usando el filtro complementario  
3. Verifica condiciones de seguridad y aplica corte si es necesario  
4. Ejecuta el PID si el robot está dentro de ángulos seguros  
5. Controla los motores según la salida del PID  
6. Envía datos al Serial Plotter  

## Archivo principal

El archivo principal del programa se encuentra como:

[Codigo/auto_balance_BB8.ino](Codigo/auto_balance_BB8.ino)
