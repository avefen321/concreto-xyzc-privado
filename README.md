# Concrete CNC XYZC 3.6.0 — despliegue público en Render

Actualización: 20-10-2026. Este repositorio contiene únicamente los archivos necesarios para ejecutar la aplicación web; no incluye documentos personales, presentaciones, resultados históricos ni archivos G-code del usuario.

## Aplicación desplegada

El frontend HTML/CSS/JavaScript y la API FastAPI se sirven desde el mismo proceso. El motor mantiene la conversión XYZC 3.6.0, curvas generales con G2/G3, círculos completos, referencia inicial C0, alternancia segura de trazos abiertos, seam para loops cerrados, importación, simulación y descargas.

Render ejecuta:

```text
Runtime: Python 3.12.14
Plan: Free
Build: pip install -r requirements-web-lock.txt
Start: python -m backend.hosted
Health: /healthz
Instances: 1
```

`backend.hosted` enlaza Uvicorn a `0.0.0.0` y al puerto indicado por `PORT`. El proceso conserva hasta ocho conversiones en memoria; se pierden cuando Render suspende, reinicia o vuelve a desplegar el servicio.

## Acceso público

`XYZC_PUBLIC_ACCESS=true` permite abrir la interfaz y la API sin usuario ni contraseña. Las respuestas dinámicas utilizan `Cache-Control: no-store`. El modo privado anterior permanece disponible como alternativa: al cambiar `XYZC_PUBLIC_ACCESS` a `false`, el arranque vuelve a exigir `XYZC_PRIVATE_USER` y una contraseña de al menos 24 caracteres.

La aplicación es una herramienta de preparación y auditoría geométrica. La verificación de software no sustituye la validación de los límites, el signo, el cero físico ni la simetría real del eje C en LinuxCNC.

## Verificación

```text
python -m pip install -r requirements-web-test.txt
python -m unittest tests.test_private_web tests.test_public_web_asgi -v
```

La prueba alojada comprueba ambos modos: el cierre de acceso privado y el acceso público explícito. En modo público valida `/healthz`, la interfaz, `/api/config` y una conversión sin cordón.

Después de desplegar se debe comprobar la URL real, el estado `live`, `/healthz`, `/api/config`, una conversión y los logs recientes.

## Plan gratuito

El servicio Free puede entrar en suspensión por inactividad y tardar en responder al primer acceso. No hay base de datos ni disco persistente. Los archivos descargados por el usuario permanecen en su equipo.
