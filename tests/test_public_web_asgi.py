"""Dependency-level smoke test for the public Render ASGI entrypoint."""
import asyncio
from importlib.util import find_spec
import json
import os
import unittest
from unittest.mock import patch

if find_spec('fastapi') is not None:
    from backend.hosted import create_app
else:
    create_app = None


async def asgi_request(application, method, path, payload=None):
    body = b'' if payload is None else json.dumps(payload).encode('utf-8')
    messages = []
    delivered = False

    async def receive():
        nonlocal delivered
        if not delivered:
            delivered = True
            return {'type': 'http.request', 'body': body, 'more_body': False}
        # Keep the receive channel open while Starlette streams the response.
        await asyncio.Event().wait()

    async def send(message):
        messages.append(message)

    headers = [(b'host', b'testserver')]
    if body:
        headers.append((b'content-type', b'application/json'))
        headers.append((b'content-length', str(len(body)).encode('ascii')))
    scope = {
        'type': 'http', 'asgi': {'version': '3.0'}, 'http_version': '1.1',
        'scheme': 'http', 'method': method, 'path': path,
        'raw_path': path.encode('ascii'), 'query_string': b'', 'headers': headers,
        'client': ('127.0.0.1', 12345), 'server': ('testserver', 80),
    }
    await application(scope, receive, send)
    start = next(message for message in messages if message['type'] == 'http.response.start')
    response_body = b''.join(message.get('body', b'') for message in messages
                             if message['type'] == 'http.response.body')
    return start['status'], dict(start.get('headers', [])), response_body


@unittest.skipIf(create_app is None, 'Install requirements-web-lock.txt to test hosted ASGI')
class PublicWebAsgiTests(unittest.TestCase):
    def test_public_health_interface_config_and_conversion(self):
        with patch.dict(os.environ, {'XYZC_PUBLIC_ACCESS': 'true',
                                     'XYZC_PRIVATE_USER': '',
                                     'XYZC_PRIVATE_PASSWORD': ''}):
            application = create_app()

        status, headers, body = asyncio.run(asgi_request(application, 'GET', '/healthz'))
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), {'status': 'ok'})
        self.assertEqual(headers[b'cache-control'], b'no-store')

        status, _, body = asyncio.run(asgi_request(application, 'GET', '/'))
        self.assertEqual(status, 200)
        self.assertIn(b'Concrete CNC', body)

        status, _, body = asyncio.run(asgi_request(application, 'GET', '/api/config'))
        self.assertEqual(status, 200)
        self.assertIn('defaults', json.loads(body))

        request = {
            'source': 'G21 G90\nM3\nG1 X100 Y0 F600\nM5\nM2\n',
            'filename': 'public-smoke.ngc',
            'config': {'cordon': {'mode': 'omit'}},
        }
        status, _, body = asyncio.run(asgi_request(application, 'POST', '/api/convert', request))
        self.assertEqual(status, 200)
        response = json.loads(body)
        self.assertNotEqual(response['status'], 'FAIL')
        self.assertEqual(response['report']['backend_version'], '3.6.0')


if __name__ == '__main__':
    unittest.main()
