# Concrete CNC XYZC — despliegue privado en Render

Preparación: 19-10-2026. Motor 3.5.0. Este documento no acredita un despliegue remoto.

## Arquitectura y arranque

Frontend HTML/CSS/JavaScript y API FastAPI servidos desde el mismo proceso. No requiere Node, un servidor de frontend, base de datos ni migrar los algoritmos a JavaScript. Las rutas relativas de la interfaz funcionan bajo un mismo origen HTTPS.

El inicio local anterior es `python run.py`, con `backend.main:app` o el servidor alternativo en localhost:8000. El inicio de alojamiento es **`python -m backend.hosted`**: ejecuta Uvicorn en `0.0.0.0`, usa el puerto que indique `PORT` y mantiene un solo worker. No se debe usar `run.py` ni `uvicorn backend.main:app` como comando de inicio remoto, porque el primero abre un navegador y el segundo omite la protección del alojamiento.

`backend/hosted.py` añade exclusivamente el límite de acceso. Los módulos del motor y los archivos del frontend se conservan. Se incluyen las dos rutas de ejemplo servidas por la aplicación.

## Acceso privado

- El navegador pide usuario y contraseña mediante HTTP Basic sobre HTTPS.
- La interfaz, recursos, ejemplos, API, documentación y descargas requieren esas credenciales. Conocer un identificador de conversión no concede acceso.
- `XYZC_PRIVATE_USER` define el usuario; `XYZC_PRIVATE_PASSWORD` requiere al menos 24 caracteres. Sin valores válidos, el arranque falla y no deja abierta la aplicación.
- `render.yaml` propone `propietario` y solicita que Render genere aleatoriamente la contraseña. El valor debe consultarse/configurarse en las variables de entorno del servicio, nunca guardarse en el repositorio ni enviarse por el chat.
- Solo `GET /healthz` y `HEAD /healthz` son públicos y devuelven un estado mínimo sin archivos, cálculos ni credenciales.
- Las respuestas de la aplicación llevan `Cache-Control: no-store`. No se habilita CORS para otros sitios.

La URL de Render utiliza un nombre accesible por Internet; **el contenido de la aplicación queda restringido por autenticación**. Esto no es una red privada/VPN de Render. Sus Private Services de red interna no ofrecen la URL de navegador solicitada. Si se requiere además que ni siquiera exista una ruta desde Internet, haría falta otra solución de acceso privado.

## Configuración preparada

Archivo: `render.yaml`.

```text
Runtime: Python 3.12.14
Plan: Free
Build: pip install -r requirements-web-lock.txt
Start: python -m backend.hosted
Health: /healthz
Instances: 1
Auto deploy: off
```

El archivo de dependencias fijadas contiene las versiones verificadas en un entorno web aislado. Las dependencias de desarrollo anteriores de `requirements.txt` se conservan aparte. Las pruebas HTTP requieren también `requirements-web-test.txt`.

Para desplegar hace falta una cuenta Render conectada y una fuente de código que Render pueda leer: un repositorio **privado** del proveedor Git vinculado, o una imagen de contenedor privada compatible. El ZIP preparado sirve para transferir el código a ese repositorio; Render no despliega un ZIP directamente. No se debe crear un repositorio público para resolver el acceso. Antes de crear un servicio se deben comprobar los recursos existentes de la cuenta para evitar duplicados, y verificar explícitamente que el plan elegido sea Free.

Este proyecto local no tiene un repositorio Git remoto configurado. La conexión del alojamiento y el acceso a la fuente se deben confirmar antes de informar una URL desplegada.

## Qué incluye el paquete

`deployment/package_for_render.py` crea un ZIP con una lista explícita de fuentes: backend, frontend, ejemplos de interfaz, configuración de Render, dependencias y prueba de acceso. No agrega la carpeta completa del proyecto, documentos personales, PDFs/PPT, archivos originales del usuario, conversiones anteriores, entornos virtuales ni secretos. El manifiesto enumera tamaño y SHA-256 de cada fuente. No sobrescribe un paquete existente.

Los archivos originales del proyecto permanecen en su sitio. El paquete es un artefacto de preparación; no es una URL ni demuestra que Render lo haya ejecutado.

## Verificación

En un entorno con las dependencias instaladas:

```text
python -m pip install -r requirements-web-test.txt
python -m unittest tests.test_private_web -v
```

Las pruebas verifican: arranque cerrado sin secretos, rechazo de credenciales incorrectas/malformadas, protección de todas las rutas, acceso autorizado sin alterar frontend, revisión de cordón, conversión paramétrica sin cordón, simulación y descarga de G-code, reporte JSON y CSV. Se debe repetir la comprobación contra la **URL real desplegada**, incluyendo rechazo sin credenciales y acceso correcto con ellas. Hasta entonces la validación remota está pendiente.

## Límites del plan gratuito

Según [Render Free](https://render.com/docs/free), consultado el 19-10-2026: el servicio se suspende tras 15 minutos sin tráfico y suele necesitar alrededor de un minuto para despertar. Hay límites de horas, tráfico y compilación; no es una garantía de funcionamiento continuo ni está recomendado por Render para producción.

No se añaden base de datos ni disco. La aplicación actual conserva hasta ocho conversiones en memoria; se pierden al reiniciar, suspender o desplegar de nuevo. Los archivos descargados en el equipo del usuario sí permanecen. No se incorporará persistencia ni se contratará capacidad de pago sin autorización.

Los cálculos exigentes pueden ser lentos o exceder los recursos gratuitos. La validación local no demuestra todavía el rendimiento de la pared de diez capas en una instancia Free. Si ese caso falla por memoria o tiempo, se informará el límite sin cambiar tolerancias ni algoritmos ni subir de plan automáticamente.

Si la cuenta tiene un medio de pago, Render puede cobrar excedentes de ciertos consumos. Elegir Free no equivale a un límite monetario universal: se deben revisar los límites de gasto antes del despliegue. No se modifican opciones de facturación ni se habilitan servicios de pago sin aprobación.

Referencias: [FastAPI en Render](https://render.com/docs/deploy-fastapi), [Blueprint YAML](https://render.com/docs/blueprint-spec), [HTTP Basic de FastAPI](https://fastapi.tiangolo.com/advanced/security/http-basic-auth/).
