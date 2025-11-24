# Proyecto-Robot-Auto-Balancin
# BB-8 AutoBalance – Proyecto Mecatrónica ME4250 (Primavera 2025)

Este repositorio contiene el desarrollo completo del proyecto de Mecatrónica ME4250, cuyo objetivo fue diseñar, fabricar e implementar un **robot auto-balanceado inspirado en R2-D2**.  
El sistema utiliza un controlador **PID** junto al sensor inercial **MPU6050** para mantener el equilibrio dinámico mediante la acción de motores NEMA17 controlados por easydrivers.

---

## Integrantes
- Santiago Ávila  
- Diego Gajardo  
- Leonardo Pizarro  
- Daniela Quiroz  

### Profesor:
Harold Valenzuela  

### Auxiliar:
Fernando Navarrete  

### Ayudantes:
Valentina Abarca  
Fernanda Echeverría  

---

## Contenido del repositorio

- `/CAD` – Archivos CAD del robot y sus estructuras internas.  
- `/Componentes` – Lista de materiales y componentes electrónicos utilizados.  
- `/Codigo` – Código Arduino completo del robot (PID + lectura IMU).  
- `/Diagramas` – Diagramas electrónicos y de control automático.  
- `/Registros` – Fotos, videos y evidencias del proceso y pruebas.  
- `/Referencias` – Bibliografía y material consultado.

---

## Resumen del proyecto

Este robot auto-balanceado implementa:

- Control PID (Proporcional–Integral–Derivativo).  
- Sensor IMU MPU6050 para medición de inclinación y velocidad angular.  
- Motores NEMA17 y drivers A4988/Easydrivers.  
- Estructura impresa en PLA con estética temática de **R2-D2**.  
- PCB personalizada para mejorar la organización del cableado interno.  

El prototipo alcanzó un equilibrio **medianamente estable**, con tendencia a caer hacia un lado debido a la necesidad de una mejor sintonización del PID y una respuesta más rápida de los motores. Aun así, se logró validar la arquitectura electrónica, mecánica y de control del sistema.

---

## Próximos pasos

- Ajuste fino del PID (Kp, Ki, Kd).  
- Optimización del centro de masa.   

---

## 📜 Licencia
Libre uso académico.  
