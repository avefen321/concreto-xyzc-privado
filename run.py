from pathlib import Path
import webbrowser
import threading
import time
from urllib.error import URLError
from urllib.request import urlopen
import json
import os
import sys


URL = "http://127.0.0.1:8000"


def server_ready():
    """Return True only when this application's local API is responding."""
    try:
        with urlopen(f"{URL}/api/config", timeout=0.4) as response:
            return response.status == 200
    except (OSError, URLError):
        return False


def open_when_ready():
    """Wait for the listening socket before asking the browser to navigate."""
    for _ in range(150):
        if server_ready():
            webbrowser.open(URL)
            return
        time.sleep(0.1)
    print(f"El servidor no respondió a tiempo. Revisa esta ventana y abre {URL} manualmente.")


if __name__ == "__main__":
    # Direct `python run.py` must use the same environment as the shortcut.
    project_python=Path(__file__).resolve().parent/'.venv-web/Scripts/python.exe'
    if project_python.is_file() and Path(sys.executable).resolve()!=project_python.resolve():
        os.execv(str(project_python),[str(project_python),str(Path(__file__).resolve()),*sys.argv[1:]])
    # Only this loopback desktop launcher enables local optimization by default.
    # backend.hosted and render.yaml remain unchanged and keep it disabled.
    os.environ.setdefault('XYZC_OPTIMIZATION_ENABLED','1')
    print(f"Concrete CNC XYZC: {URL}")
    if server_ready():
        try:
            with urlopen(f'{URL}/api/optimization/capabilities',timeout=2) as response:
                existing=json.load(response)
        except (OSError,ValueError):
            existing={}
        if existing.get('integration_version')!='2.2' or not all(existing.get('algorithms',{}).get(k) for k in ('bo','tpe')):
            print('Hay un servidor anterior abierto en el puerto 8000; no se ha reemplazado.')
            print('Cierre su ventana de Python/servidor y vuelva a ejecutar ABRIR_PROGRAMA.bat.')
            print('Esa instancia no tiene los algoritmos o los cambios actuales cargados.')
            raise SystemExit(2)
        print("El servidor ya estaba iniciado. Abriendo la aplicación.")
        webbrowser.open(URL)
        raise SystemExit(0)
    threading.Thread(target=open_when_ready, daemon=True).start()
    try:
        import uvicorn
        import fastapi
    except ImportError:
        from backend.fallback_server import run
        run()
    else:
        uvicorn.run("backend.main:app", host="127.0.0.1", port=8000, reload=False)
