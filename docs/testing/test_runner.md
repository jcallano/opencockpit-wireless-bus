# OpenCockpit Wireless Bus - Test Runner

Herramienta de pruebas automatizadas para validar el bus wireless (Node A + Nodes
perifericos) segun la estrategia de tests de la especificacion (seccion 14).

## Requisitos

- Python 3.x
- Puerto serie del Node A (USB CDC)
- Firmware con soporte MSG_TEST en Node A y Node C

## Ubicacion

`software/tools/test_runner.py`

## Que valida

- Discovery: que los nodos registren en el tiempo limite.
- Latency: RTT de mensajes MSG_TEST.
- Throughput: tasa de mensajes MSG_TEST recibidos por segundo.

## Uso basico

```bash
python software/tools/test_runner.py --port /dev/ttyACM0 --node-id 0x02
```

### Parametros principales

- `--port`: puerto serie del Node A (obligatorio).
- `--node-id`: ID del nodo destino para pruebas (default 0x02).
- `--expected-nodes`: cantidad de nodos esperados en discovery (default 2).
- `--discovery-timeout`: timeout de discovery en segundos (default 5.0).
- `--latency-samples`: cantidad de muestras de RTT (default 50).
- `--latency-timeout`: timeout por muestra en segundos (default 0.2).
- `--throughput-count`: cantidad de mensajes a enviar (default 500).
- `--throughput-timeout`: timeout total de throughput en segundos (default 5.0).
- `--payload-size`: tamano del payload de prueba (default 16).
- `--verbose`: imprime payloads de discovery.

## Criterios de exito (referencia)

Basado en la especificacion:

- Discovery: todos los nodos dentro de 5s.
- Latency: P99 < 5ms bajo carga.
- Throughput: >= 500 msg/s sin drops.

## Notas

- El RTT se mide desde el envio hasta recibir MSG_TEST_RSP.
- Si usas otro node_id, asegure que el nodo implemente MSG_TEST.
- Asegure que Node A este en modo CDC+SLIP.
- Los logs de Node A van por UART0 (pins 44/43); no usar CDC para logs mientras corre SLIP.
- Evita resetear Node A durante la prueba (el puerto CDC puede desconectarse en la VM).
