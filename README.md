# Concrete CNC · XYZC Studio

Aplicación local para convertir G-code XYZ en movimientos coordinados `G1/G2/G3 XYZC` para una boquilla rectangular rotativa. Conserva arcos circulares y fillets nativos, ajusta curvas libres mediante arcos/biarcos con tolerancias y conserva G1 adaptativo donde el ajuste no es válido. Normaliza por defecto las alturas de capa a Z20/Z18. LinuxCNC sigue controlando los ejes físicos.

Versión del backend: **3.7.0** (23-10-2026).

La integración local de optimización **2.2** (28-10-2026) incorpora una tabla de candidatas con recomendación explícita, visualización individual y descarga directa de cada G-code XYZC. La simulación 3D utiliza Three.js local para mostrar el depósito progresivo, las alturas de capa configuradas, los viajes sin material y los ejes XYZ. Véanse el [manual actualizado](docs/xyzc_integration/MANUAL_USO.md) y el [informe de validación](docs/xyzc_integration/candidatas_y_3d_2026-10-28/INFORME_Y_USO.md).

Las curvas orgánicas tienen un nuevo modo opcional: **Reconstrucción paramétrica acotada**. XYZ y C se calculan a partir de una misma curva Hermite/Bezier; después se aproximan mediante arcos/biarcos con unión sobre esa curva y G1 verificados. El modo **Compatible** sigue seleccionado por defecto. El nuevo modo cambia explícitamente la referencia angular de C; no reproduce el PCHIP anterior. Véanse [investigación previa](GENERAL_CURVES_RESEARCH_PLAN_2026-10-19.md), [implementación y resultados](GENERAL_CURVES_IMPLEMENTATION_REPORT_2026-10-19.md) y [preparación de la optimización posterior](GENERAL_CURVES_OPTIMIZATION_SPEC_2026-10-19.md).

Las curvas libres ahora prueban biarcos con proporción variable, unión compatible con C y comprobación continua de error. El refinamiento adaptativo conserva la polilínea/campo C de referencia y compara el coste de bloques antes de aceptar subdivisiones. Véanse [investigación y plan](COMPLEX_CURVE_ARCFIT_PLAN.md) y [resultados de la mejora](COMPLEX_CURVE_ARCFIT_REPORT.md). Los fillets conservan sus algoritmos.

Desde 3.7.0, el operador puede homear `C0` con la boquilla paralela a `+X`, `+Y`, `−X` o `−Y`. El programa llega primero al punto inicial manteniendo esa referencia, ordena un movimiento exclusivamente rotativo `G1 C…`, fuerza una parada exacta `G61.1`, restaura el modo de trayectoria y solo después inicia el recorrido. Así, un triángulo o una curva pueden comenzar con cualquier inclinación sin homear manualmente a 60°, 120° u otro ángulo incómodo. Los modos anteriores —home ya tangente y modelo absoluto— siguen disponibles. `G90` se conserva porque define coordenadas absolutas; no significa un giro de 90°. La simetría bidireccional de 180° elige el giro equivalente más corto. Toda subida de capa mantiene X, Y y C constantes.

- [Auditoría previa de arcos nativos](NATIVE_ARC_AUDIT_BEFORE.md)
- [Implementación y resultados de arcos nativos](NATIVE_ARC_IMPLEMENTATION_REPORT.md)

- [Auditoría previa de sobremuestreo](CORNER_SAMPLING_AUDIT_BEFORE.md)
- [Informe de optimización global de curvas y Z](CORNER_SAMPLING_OPTIMIZATION_REPORT.md)
- [Informe de la optimización anterior](ASTRA_BACKEND_OPTIMIZATION_REPORT.md)

## Inicio

Requiere Python 3.12 o posterior. En una instalación normal:

```bash
python -m pip install -r requirements.txt
python run.py
```

Abrir <http://127.0.0.1:8000>. En Windows, el inicio recomendado es hacer doble clic en `ABRIR_PROGRAMA.bat`; también se puede ejecutar `./start.ps1`. El archivo `index.html` de la raíz informa el estado, pero un navegador no puede iniciar Python por motivos de seguridad. Si FastAPI y Uvicorn no están instalados, `run.py` usa automáticamente un servidor HTTP local incluido en el proyecto. La matemática y la interfaz funcionan sin conexión a Internet.

Para un alojamiento externo con acceso privado se agregó un inicio separado, `python -m backend.hosted`, y una configuración Free de Render. Requiere credenciales de acceso en las variables de entorno y mantiene el motor y el frontend originales. Véase [preparación del despliegue privado](deployment/README_RENDER.md). La existencia de estos archivos no significa que ya haya una URL desplegada; el alojamiento y las credenciales del proveedor deben estar conectados y el servicio validado.

## Flujo de uso

1. Cargar un archivo `.ngc`, `.gcode`, `.nc` o `.txt` en **Archivo**.
2. Revisar capas, trazos, rangos y avisos del parser.
3. Ajustar radio, muestreo, orientación, normalización Z y límites del eje C en **Conversión**.
4. Convertir y revisar **Simulación** y **Auditoría**.
5. Descargar `pieza_xyzc.ngc`, `conversion_report.json` y `samples.csv`.

La pantalla **Archivo** tiene un botón para cargar el cuadrado de ejemplo. El ejemplo principal es [cuadrado_300_6_capas.ngc](examples/cuadrado_300_6_capas.ngc). También se incluyen [inicio_travel_alto.ngc](examples/inicio_travel_alto.ngc), [tres_capas_cuadrado.ngc](examples/tres_capas_cuadrado.ngc) y [dos_capas.ngc](examples/dos_capas.ngc).

Para probar curvas complejas, pulsa **Cargar contorno orgánico · modo paramétrico** en Archivo. Ese botón carga [contorno_organico.ngc](examples/contorno_organico.ngc), selecciona el nuevo modelo y propone seis decimales XYZ/C/IJ. También puedes elegirlo en **Conversión → Referencia de curvas complejas → Modelo de curva → Nuevo · reconstrucción paramétrica acotada**. Mantén activados **Exportar G2/G3** y **Ajustar curvas libres**. Al convertir puedes elegir **Sin cordón**. La auditoría muestra la cobertura de arcos y la referencia angular utilizada. `PRESERVE_PATH`, las opciones desactivadas y los eventos protegidos mantienen el comportamiento compatible donde corresponde.

La cobertura de arcos mejoró del 4,20 % al 95,38 % en la pared senoidal de prueba, manteniendo los límites configurados para las regiones reconstruidas. El nuevo modelo puede generar más bloques: en esa prueba pasó de 1240 a 6940. No es todavía una optimización del tamaño del archivo. Los resultados reproducibles están en [resultados_curvas_generales_2026-10-19](resultados_curvas_generales_2026-10-19/).

## Pipeline y arquitectura

```text
G-code XYZ → movimientos canónicos → capas y trazos
→ normalización Z semántica → clasificación geométrica
→ geometría imprimible → ajuste de arcos / biarcos → tangentes analíticas y C(s)
→ llegada al inicio con C0 físico → alineación inicial C-only con parada exacta
→ primitivas de máquina + muestras visuales → diagnóstico cinemático
→ FinalMotionPlan → serializer híbrido G1/G2/G3, CSV, reporte y visor
```

- `backend/core/gcode.py`: parser modal y geometría analítica de arcos G17.
- `backend/core/path.py`: capas, trazos, Z-only/travel, compresión colineal y arcos nativos.
- `backend/core/layer_z.py`: clasificación y normalización semántica de Z.
- `backend/core/fillet.py`: eventos de esquina, selección adaptativa de radio y fillets analíticos.
- `backend/core/sampling.py`: contrato común `CurveSampler`, `ArcSampler` y `SmoothCurveSampler`.
- `backend/core/arc_fitting.py`: ajuste progresivo, refinamiento por error, biarcos Hermite, inflexiones, merge y fallback.
- `backend/core/parametric.py`: referencia Hermite/Bezier acotada, tangentes cíclicas e inflexiones.
- `backend/core/curve_reconstruction.py`: biarcos con unión sobre la curva, cotas polinómicas y G1 certificados del nuevo modo.
- `backend/core/primitives.py`: movimientos LINE/ARC/Z_ONLY/TRAVEL y cinemática analítica, separados de la pantalla.
- `backend/core/arc_serialization.py`: selección R/IJ y validación del arco después del redondeo.
- `backend/core/orientation.py`: tangentes, heading, simetría, unwrap y orientación continua.
- `backend/core/simplification.py`: reducción con límites simultáneos de error geométrico y angular.
- `backend/core/diagnostics.py`: densidad G1, longitudes de bloque, microbloques y variación de C.
- `backend/core/conversion.py`: coordinación, validación, serialización y `FinalMotionPlan`.
- `backend/core/validators.py`: verificaciones independientes para rectas, curvas, Z y salida redondeada.
- `backend/main.py`: API FastAPI; `backend/fallback_server.py`: mismo flujo HTTP local sin dependencias externas.
- `frontend/`: configuración, comparación legacy, visor 2D/3D, esquinas y gráficas.

El núcleo no importa FastAPI ni código de interfaz. Los tests se ejecutan con:

```bash
python -m unittest discover -s tests -v
python -B -m tests.corner_sampling_benchmarks
python -B -m tests.general_curve_benchmarks
```

## Entrada y semántica

Se reconocen `G0/G1/G2/G3`, `G20/G21`, `G90/G91`, `G90.1/G91.1`, `G92`, `G17/G18/G19`, `G94`, `M3/M4/M5`, `M82/M83`, `M2/M30`, `F`, `C` y `E`. Los arcos G17 se conservan como geometría analítica hasta la exportación nativa, incluidos barridos mayores de 180°, hélices y vueltas P. Los arcos en otros planos generan error explícito. `G93` y `G95` se rechazan porque su semántica de feed no está implementada. `G92` cambia el origen lógico pero no crea movimiento. `G28/G30` se bloquean porque el postprocesador no conoce la posición física de homing. Internamente XYZ y E se vuelven absolutos en milímetros. C de entrada se registra; durante deposición se vuelve a calcular desde la tangente final y la alineación configurada.

Si no hay comandos de proceso ni E, se tratan los `G1` como deposición. Si hay `M3/M4/M5`, estos determinan el estado del proceso. Los `G0` son travel. Un Z-only corta el trazo sin generar heading. Una rampa XYZ con Z variable puede continuar en la misma capa.

## Orientación y esquinas

`C = heading XY + alineación + calibración`. La alineación predeterminada es **90°**, de modo que el eje largo de la boquilla queda perpendicular a la tangente. El período angular se puede poner en 180° o 360° y C permanece continuo, incluso más allá de una vuelta. El modo compatible usa derivada centrada respecto a longitud de arco e interpolación cúbica PCHIP sin overshoot, sin cruzar trazos o esquinas clasificadas. En el nuevo modo, las regiones reconstruidas usan la derivada de su propia curva paramétrica. La continuidad de esa referencia es C1 dentro de los tramos suaves, no C2 global.

Para el inicio con referencia fija, si `H` es el ángulo físico de la boquilla cuando LinuxCNC indica `C0`, se usa `C_inicio = equivalente_más_cercano(ángulo_boquilla − H, C_actual, período)`. Los valores de `H` son 0° para +X, 90° para +Y, 180° para −X y −90° para −Y. Este desplazamiento se aplica a todo el campo C, no solo al primer punto, por lo que la continuidad posterior y los algoritmos de curvas/fillets permanecen intactos. La velocidad inicial es explícita en °/s y el bloque G94 se exporta en °/min; si existe un límite de velocidad C menor, se usa ese límite.

En **SMOOTH_CORNERS**, opción predeterminada, cada esquina factible genera una primitiva circular. El radio preferido parte de la longitud de boquilla de 30 mm y se limita por longitud disponible, desviación geométrica, radio máximo y, si está configurada, velocidad de C. C se calcula sobre la trayectoria optimizada; en T1 conserva el heading entrante y en T2 alcanza el saliente. Cada fillet se exporta como un G2/G3 con C. Una esquina no factible queda sin fillet y se informa como `FILLET_NOT_FEASIBLE`. Las curvas libres utilizan arcos/biarcos solo si cumplen simultáneamente las tolerancias; pueden conservar regiones G1.

En **PRESERVE_PATH** se conserva XYZ exacto. Allí sigue disponible la estrategia de giro anticipado `h(u)=10u³−15u⁴+6u⁵`, con las mismas opciones cinemáticas y `EXACT_VERTEX` del prototipo anterior.

En los fillets, el backend evalúa XYZ, tangente y curvatura analíticamente. El sampler decide cada intervalo por error geométrico o de cuerda, error de interpolación C, cambio máximo de C y longitud máxima del bloque. Los valores iniciales de laboratorio son 0,30 mm, 1°, 15° y 10 mm; son tolerancias configurables, no pasos fijos. Las curvas suaves usan el mismo contrato para simplificar o refinar. La salida usa `G21 G90`, con C en el mismo bloque que XYZ, y vuelve a comprobar los bloques redondeados.

## Simplificación, capas y feed

La simplificación predeterminada conserva las fronteras y usa tolerancias de 0,30 mm y 1°. Las rectas con C/F/Z/proceso constantes se comprimen por una vía exacta. En `PRESERVE_PATH` también se aplica la tolerancia XY de preservación. El reporte indica cantidades antes/después y errores medidos.

**Elegir inicio seguro** es opcional: en loops cerrados uniformes selecciona una recta con guard físico de 20 mm, inserta un punto sobre ella y cambia cíclicamente el inicio. No traslada el contorno. Cuando no hay candidato válido informa el motivo. Un movimiento Z-only mantiene las coordenadas XY del seam imprimible.

F se conserva por defecto (`PRESERVE_SOURCE`). `RECOMMEND` informa el máximo por C sin cambiar el archivo. `ADAPTIVE_EXPERIMENTAL` usa los límites C configurados y comprueba omega/alpha antes de exportar. Sin límites reales conserva F; si F es desconocida, no la inventa. El feed final se ve en cada muestra y en CSV.

G-code, CSV y visor se derivan de un único `FinalMotionPlan`. El G-code consume `primitives`; el CSV incluye muestras visuales con `display_only` y `emit` para distinguirlas de los bloques exportados. El visor interpola geometría y C de las primitivas entregadas por backend. Se conservan origen, capa/trazo, motivo, tangente, vector de boquilla, hashes y mapeo por línea/primitiva.

## Normalización Z

La conversión está activa por defecto y usa `Z_target(k) = 20 + 18·k mm`, con `k` desde cero. Seis capas quedan en `20, 38, 56, 74, 92, 110 mm`. La transformación opera sobre capas detectadas, no sobre texto. Una transición conserva X/Y; un Z-only no altera tangente ni C; un `MACHINE_SAFE_Z` explícito se preserva. El reporte registra Z de origen, destino, motivo y error por capa.

## Cordón inicial y revisión de cierre antes de convertir

Al pulsar **Convertir**, aparece primero una ventana para elegir el tratamiento del cordón de arranque:

- **Añadir cordón de 65 mm:** crea un G0 hasta un punto exterior diagonal a 45° y un G1 de 65 mm hasta el inicio del contorno. Si hay un cordón diagonal existente, lo ajusta a 65 mm. Se ve como un trazo separado en la simulación. El reporte comprueba longitud, unión y orientación C antes del contorno.
- **Mantener cordón:** conserva un cordón existente detectado. Si no hay uno, la opción se desactiva. En programas cuyo estado de material impide ajustar con seguridad, se ofrece conservar el recorrido original.
- **Sin cordón:** omite un cordón detectado o una aproximación inicial que no pertenece a la figura. Si hay un M3, desplaza el G0 previo al inicio del contorno y retira el tramo previo, cuando se puede verificar esa secuencia.

La ventana muestra el punto propuesto y avisa si requiere X o Y negativos. En el cuadrado de 300 mm adjunto, el cordón exterior propuesto empieza en X 104,04, Y −45,96 mm y termina en X 150, Y 0 mm: comprueba el alcance de la máquina antes de usar ese archivo. La ubicación es una propuesta geométrica; el programa no conoce los límites físicos de los ejes.

La extrusión de concreto es independiente y manual. Por defecto, el G-code exportado contiene el movimiento XYZC, el avance F y el fin de programa; no incluye activaciones de material como M3/M4/M5, valores E/S ni otros M auxiliares aunque aparezcan en el archivo de entrada. El cordón es solo la trayectoria de 65 mm para asentar el flujo que tú controlas. Para compatibilidad con otra máquina, una llamada directa al backend puede solicitar `config.serialization.motion_only = false`; la interfaz normal utiliza el valor seguro `true`.

Después de elegir el cordón, si la primera capa queda abierta y al menos dos capas siguientes confirman el mismo contorno, sentido, orden y regreso XY/Z en un seam estable, aparece **Primera capa: falta de cierre**. La ventana muestra la longitud pendiente y la cantidad de referencias, y espera una elección:

- **Completar cierre:** distingue la aproximación inicial, deposita cada regreso existente a la altura inferior antes de subir, y añade una sola vez el regreso final. En cruz y hexágono se cierran las 30 capas, con 29 transiciones verticales. Se conserva F de origen; el cierre final usa la última F activa.
- **Mantener recorrido:** continúa con el comportamiento anterior y registra el cierre pendiente como aviso en Capas y en el reporte.

Escape cancela la conversión pendiente. Ambas elecciones se solicitan en cada conversión y se incluyen en su configuración/hash. El cuadrado adjunto de seis capas ya cierra y no muestra la segunda pregunta ni recibe cierres añadidos.

La comparación admite subdivisiones collineales y arcos nativos subdivididos sobre el mismo círculo. La reparación actual requiere un regreso G1 confirmado en las referencias; conserva las curvas del contorno. No reconstruye cierres curvos/compuestos ni cambios de seam entre capas: conserva esos casos, rampas, varios trazos, capas diferentes, eventos bloqueados o deposición/volumen explícitos M/E/S y explica el motivo. No cierra cualquier trazo abierto mediante una recta.

`POST /api/files/closure-analysis` usa el mismo detector que la conversión y entrega los datos del cordón. Las llamadas directas a `/api/convert` conservan el comportamiento previo por defecto; para elegir cordón usa `config.cordon.mode = "add"`, `"keep"` u `"omit"`, y para el cierre `config.layer_completion.decision = "complete"` (`"preserve"` para mantener). La tolerancia de comparación de capas predeterminada es 0,001 mm, y se requieren como mínimo dos referencias. `report.cordon` y `report.layer_completion` registran ambas decisiones. El cordón, el cierre y las subidas se verifican sobre el plan imprimible; G-code, CSV y visor derivan de ese plan.

Las salidas anteriores de validación de cierre se conservan en `resultados_cierre_capas/`. El backend actualizado es 3.7.0. Si había una sesión anterior abierta, reinicia `start.ps1` y recarga la página para usar la nueva versión.

## Precisión de salida y guardado

Los campos **Decimales XYZ**, **Decimales C** y **Decimales F** determinan exactamente cuántos dígitos aparecen en el G-code. Con XYZ a 3 decimales, por ejemplo, una coordenada se escribe `X277.765`, no `X277.7645714`; con 4 se escribe `X277.7646`. Los cálculos y la comparación con la figura conservan su precisión interna. La auditoría de serialización informa por separado el error máximo de redondeo; con 3 decimales el error por eje no supera 0,0005 mm. La tolerancia XY de configuración sigue controlando la geometría interna.

En la reconstrucción paramétrica se proponen **6 decimales XYZ/C/IJ**, visibles y editables. Un error de coordenadas pequeño puede cambiar mucho la dirección de un G1 muy corto; por eso también se comprueba C contra la tangente del G-code realmente redondeado. Si esa precisión no basta, se rechaza el tramo o la salida con el motivo correspondiente; los decimales elegidos no se aumentan silenciosamente durante la exportación. Ejemplo de configuración API: [parametric_config.json](resultados_curvas_generales_2026-10-19/parametric_config.json).

Al pulsar **G-code XYZC**, Chrome y otros navegadores compatibles abren el selector del sistema para elegir carpeta y nombre antes de guardar. Si el navegador no ofrece esa función, el enlace conserva la descarga normal y muestra cómo usar **Guardar enlace como** o activar **Preguntar dónde guardar cada archivo** en la configuración del navegador.

## Auditoría

Se verifica cada muestra contra su segmento original, también después de redondear el XYZ que se exportará. En `SMOOTH_CORNERS` se permite únicamente la desviación local configurada. Por capa se informa bbox, centroide ponderado por longitud, longitud del path, desviación XY máxima/RMS y drift del centroide; las capas con la misma proyección XY se comparan entre sí. Cada fillet informa radio, T1/T2, centro, barrido, C inicial/final, error C en T2 y distancia de asentamiento. La auditoría bloquea el G-code ante errores geométricos o coordenadas no finitas. La demanda de velocidad y aceleración C es estimada sobre los movimientos finales; sin límites configurados su estado es **NOT CONFIGURED**.

## Limitaciones

En el programa local, **Configuración → Margen respecto de X=0 e Y=0** activa por defecto un desplazamiento XY automático de 20 mm. Tras decidir el cordón y el cierre de capas, el conversor calcula los extremos de todo el recorrido, incluidos los arcos entre puntos G2/G3. Si el mínimo de un eje toca 0 o es negativo, traslada toda la trayectoria en ese eje hasta que el mínimo quede en +20 mm; el otro eje solo se desplaza si también lo necesita. Simulación, auditoría y G-code usan las mismas coordenadas finales. La auditoría indica el desplazamiento y el X/Y máximo resultante, que debe caber dentro del recorrido físico de la máquina. La opción puede desactivarse; las llamadas directas al backend conservan el comportamiento anterior salvo que se envíe `xy_boundary_offset.enabled=true`.

**Aproximación inicial solo XY** está activada por defecto en la interfaz local. Sustituye los viajes iniciales de origen, incluidos ascensos y descensos de seguridad Z, por un único G0 al inicio elegido que solo ordena X/Y/C. El operador debe colocar físicamente la máquina en la Z de la primera capa antes de ejecutar; el valor aparece en el G-code y en Auditoría. Las subidas entre capas y la retirada final conservan su Z. El selector de inicio para loops cerrados compara los puntos medios de las rectas seguras y elige el más cercano a (0,0), con 40 mm de separación preferida a fillets y 20 mm mínimos por defecto. Si solo cabe el mínimo, registra una advertencia. Si el contorno cerrado solo tiene curvas, elige el punto suave disponible más cercano al origen y lo advierte. En el backend directo la aproximación XY es optativa (`initial_xy_approach.enabled=true`) para conservar perfiles históricos. La selección cíclica permanece anclada cuando se elige un cordón que debe unirse al inicio del contorno.

- La vista 3D utiliza Canvas con cámara ortográfica interactiva: arrastrar para orbitar, activar **Modo pan** (o usar Mayús o botón derecho) para desplazar, rueda para zoom y botones de reset, superior, frontal, lateral, isométrica y encuadre. Slider/reproducción conservan la cámara. Clic o número de muestra abre el inspector; debug permite revisar clasificación, T1/T2/centro, seam y transición Z.
- La detección de capa se orienta a impresión plana; trayectorias helicoidales quedan en su trazo actual.
- Solo se interpretan arcos G17 XY. Los arcos G18/G19 producen error explícito.
- La velocidad angular estimada es la demanda programada, no una medición del motor. Los fillets requieren validación con las capacidades reales del eje C. Una esquina `EXACT_VERTEX` en `PRESERVE_PATH` requiere parada y giro C-only; su tiempo real depende de LinuxCNC.
- `SMOOTH_CORNERS` con extrusión `E` se rechaza hasta implementar la redistribución de volumen sobre los arcos; use `PRESERVE_PATH` para esos archivos.
- El modo predeterminado genera y audita la preparación C en el punto inicial. Con la salida normal `motion_only`, la mezcla se controla manualmente: debe iniciarse después de ese giro. Si se preservan M3/M4 y aparecen antes de la aproximación, la conversión se bloquea en vez de orientar con el proceso activo.
- El visor recibe muestras visuales y primitivas de máquina; el G-code recibe únicamente las primitivas. Un resultado grande requiere más memoria local. Las gráficas consumen valores del backend, incluida F, sin volver a calcular orientación o cinemática en JavaScript.

Validar las salidas en LinuxCNC y en la máquina antes de imprimir. El programa no conecta ni controla el motor directamente.

## Resultados reproducibles

El simulador ofrece dos modos de reproducción: **Continua según F** (predeterminado) y **Por bloques G-code**. El primero interpola visualmente entre los extremos de cada bloque XYZC y usa el tiempo ideal `60·longitud/F`; el segundo avanza por los puntos a 2 bloques por segundo a 1×. Las velocidades visuales de 0,1× a 10× permiten observar el recorrido con calma. Los filtros de capa/trazo también delimitan la reproducción y al terminar se pausa.

Los tiempos G0 o sin F se marcan como estimaciones visuales. Los movimientos C-only con F conocida se animan usando grados/minuto. La reproducción no modela aceleración ni blending real de LinuxCNC. Los puntos, feed, orientación y G-code del backend se conservan; los fotogramas interpolados solo sirven para mostrar el movimiento. La regresión de duración independiente de la densidad se ejecuta con `node tests/test_simulation_playback.mjs`.

La validación actual reúne **164 pruebas de Python**. En el entorno del motor se aprobaron 156 y se omitieron 8 que requieren las dependencias web; esas 8 pruebas de alojamiento se aprobaron por separado en el entorno web. También se aprobaron las regresiones de reproducción y arcos nativos en Node. La orientación inicial tiene 8 pruebas dedicadas, y los 14 casos de cordón, capas y compatibilidad histórica volvieron a aprobarse después del cambio. Registro actual: [orientación inicial 3.7.0](INITIAL_ORIENTATION_TEST_RESULTS_2026-10-23.txt). Se conservan además los registros históricos de [curvas generales](GENERAL_CURVES_TEST_RESULTS_2026-10-19.txt) y [acceso privado](PRIVATE_WEB_TEST_RESULTS_2026-10-19.txt). Los resultados y salidas históricos quedan en:

- [CORNER_SAMPLING_BASELINE.json](CORNER_SAMPLING_BASELINE.json)
- [CORNER_SAMPLING_BENCHMARK_RESULTS.json](CORNER_SAMPLING_BENCHMARK_RESULTS.json)
- [resultados_curvas_z20_z18](resultados_curvas_z20_z18)

El benchmark histórico del perfil G1 reducía 2616 G1 legacy a 168 G1 adaptativos en seis capas. Desde 3.4.0, un fillet R30 de 90° se exporta en un único G2/G3, manteniendo seis puntos para dibujarlo. Las métricas históricas 3.4.1 están en `COMPLEX_CURVE_BENCHMARK_RESULTS.json` y sus salidas en `resultados_curvas_ij_2026-10-06/`. Las métricas actuales 3.5.0 están en `GENERAL_CURVES_BENCHMARK_RESULTS_2026-10-19.json` y `resultados_curvas_generales_2026-10-19/`. Las métricas y ejemplos anteriores se conservan en `NATIVE_ARC_BENCHMARK_RESULTS.json` y `resultados_arcos_nativos_2026-10-06/`.


## Optimización local de bloques — plan maestro v2 (26-10-2026)

Se incorporó un constructor G1/G2/G3 por programación dinámica con referencia geométrica/angular congelada, verificación de la salida y búsqueda opcional BO/GP/EI, aleatoria o TPE. El panel aparece en Conversión después de generar una referencia paramétrica. Reinicie el servidor local para cargar los cambios.

Las dependencias están separadas en `requirements-optimization-lock.txt`; el conversor básico sigue funcionando sin ellas. `run.py` habilita los trabajos locales. La entrada alojada y Render no se habilitan automáticamente para optimización.

Documentación: [manual local](docs/xyzc_integration/MANUAL_USO.md), [informe computacional](docs/xyzc_integration/REPORTE_FINAL_COMPUTACIONAL.md) y [estado de fases](docs/xyzc_integration/ESTADO_FASES.md). El piloto no demuestra superioridad de BO frente a aleatorio/TPE ni rendimiento físico: consulte sus fallos y límites antes de interpretar porcentajes.
# Actualización local de IA · 28-10-2026

El acceso directo utiliza el entorno científico `.venv-web`. BO y TPE se diagnostican desde el panel de Conversión; la búsqueda también admite el modo compatible y PRESERVE_PATH con compactación conservadora del plan actual. Para explorar representaciones de curvas se elige la referencia paramétrica antes de convertir. Véase el [informe de reparación y uso](docs/xyzc_integration/reparacion_2026-10-28/INFORME_REPARACION.md), con 210 pruebas aprobadas y verificación desde el navegador.

