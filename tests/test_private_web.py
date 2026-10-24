"""Hosted access boundary and unchanged end-to-end converter contract."""
import base64
from importlib.util import find_spec
import os
from pathlib import Path
import unittest
from unittest.mock import patch

if find_spec('fastapi') is not None and find_spec('httpx') is not None:
    from fastapi.testclient import TestClient
    from backend.hosted import create_app
else:
    TestClient = None


@unittest.skipIf(TestClient is None, 'Install requirements-web-lock.txt and httpx to test hosted ASGI')
class PrivateWebTests(unittest.TestCase):
    username = 'test-owner'
    password = 'test-only-secret-with-more-than-24-characters'

    def setUp(self):
        with patch.dict(os.environ, {'XYZC_PUBLIC_ACCESS': 'false',
                                     'XYZC_PRIVATE_USER': self.username,
                                     'XYZC_PRIVATE_PASSWORD': self.password}):
            self.application = create_app()
        self.client = TestClient(self.application)
        self.client.__enter__()

    def tearDown(self):
        self.client.__exit__(None, None, None)

    def test_missing_or_weak_secrets_fail_closed(self):
        for username, password in (('', self.password), (self.username, ''), ('bad:name', self.password),
                                   (self.username, 'short'), ('bad\nname', self.password)):
            with patch.dict(os.environ, {'XYZC_PUBLIC_ACCESS': 'false',
                                         'XYZC_PRIVATE_USER': username,
                                         'XYZC_PRIVATE_PASSWORD': password}):
                with self.assertRaises(RuntimeError):
                    create_app()

    def test_explicit_public_mode_needs_no_credentials(self):
        with patch.dict(os.environ, {'XYZC_PUBLIC_ACCESS': 'true',
                                     'XYZC_PRIVATE_USER': '',
                                     'XYZC_PRIVATE_PASSWORD': ''}):
            public_client = TestClient(create_app())
            with public_client:
                self.assertEqual(public_client.get('/healthz').json(), {'status': 'ok'})
                self.assertEqual(public_client.get('/').status_code, 200)
                config = public_client.get('/api/config')
                self.assertEqual(config.status_code, 200)
                self.assertIn('defaults', config.json())
                converted = public_client.post('/api/convert', json={
                    'source': 'G21 G90\nM3\nG1 X100 Y0 F600\nM5\nM2\n',
                    'filename': 'public-smoke.ngc',
                    'config': {'cordon': {'mode': 'omit'}}})
                self.assertEqual(converted.status_code, 200)
                self.assertNotEqual(converted.json()['status'], 'FAIL')
                self.assertEqual(converted.json()['report']['backend_version'], '3.7.0')

    def test_every_application_surface_requires_authentication(self):
        for path in ('/', '/css/app.css', '/js/app.js', '/examples/contorno_organico.ngc',
                     '/api/config', '/docs', '/openapi.json', '/api/simulate/unknown',
                     '/api/conversion/unknown/gcode', '/api/conversion/unknown/report',
                     '/api/conversion/unknown/samples', '/unknown'):
            response = self.client.get(path)
            self.assertEqual(response.status_code, 401, path)
            self.assertIn('Basic realm=', response.headers['www-authenticate'])
            self.assertEqual(response.headers['cache-control'], 'no-store')
        response = self.client.post('/api/convert', json={'source': 'G1 X1 F600'})
        self.assertEqual(response.status_code, 401)

    def test_wrong_and_malformed_credentials_rejected(self):
        for header in ('Bearer anything', 'Basic !invalid!', 'Basic ' + base64.b64encode(b'no-colon').decode(),
                       'Basic ' + base64.b64encode(b'\xff:invalid').decode()):
            self.assertEqual(self.client.get('/', headers={'Authorization': header}).status_code, 401)
        for credentials in ((self.username, 'wrong'), ('wrong', self.password)):
            self.assertEqual(self.client.get('/', auth=credentials).status_code, 401)

    def test_health_reveals_no_program_or_credentials(self):
        response = self.client.get('/healthz')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json(), {'status': 'ok'})
        self.assertNotIn(self.password, response.text)
        self.assertEqual(self.client.post('/healthz').status_code, 401)

    def test_authorized_interface_and_assets_are_unchanged(self):
        for path, local in (('/', 'frontend/index.html'), ('/js/app.js', 'frontend/js/app.js'),
                            ('/css/app.css', 'frontend/css/app.css')):
            response = self.client.get(path, auth=(self.username, self.password))
            self.assertEqual(response.status_code, 200)
            self.assertEqual(response.content, Path(local).read_bytes())
            self.assertEqual(response.headers['cache-control'], 'no-store')

    def test_full_conversion_and_downloads_without_cordon(self):
        auth = (self.username, self.password)
        source_response = self.client.get('/examples/contorno_organico.ngc', auth=auth)
        self.assertEqual(source_response.status_code, 200)
        request = {'source': source_response.text, 'filename': 'contorno_organico.ngc', 'config': {
            'arc_fit': {'reference_model': 'PARAMETRIC_HERMITE'},
            'serialization': {'xyz_decimals': 6, 'c_decimals': 6, 'ijk_decimals': 6},
            'cordon': {'mode': 'omit'}}}
        for route in ('/api/files/parse', '/api/files/closure-analysis'):
            self.assertEqual(self.client.post(route, json=request, auth=auth).status_code, 200)
        converted = self.client.post('/api/convert', json=request, auth=auth)
        self.assertEqual(converted.status_code, 200)
        result = converted.json()
        self.assertNotEqual(result['status'], 'FAIL', result['report']['errors'])
        self.assertTrue(result['report']['curve_reconstruction']['applied'])
        self.assertGreater(result['report']['native_arcs']['native_arc_length_fraction'], .9)
        key = result['id']
        simulation = self.client.get('/api/simulate/' + key, auth=auth)
        self.assertEqual(simulation.status_code, 200)
        self.assertEqual(simulation.json()['conversion_id'], key)
        for suffix in ('gcode', 'report', 'samples'):
            route = f'/api/conversion/{key}/{suffix}'
            response = self.client.get(route, auth=auth)
            self.assertEqual(response.status_code, 200)
            self.assertTrue(response.content)
            self.assertIn('attachment;', response.headers['content-disposition'])
            if suffix == 'gcode':
                self.assertIn('G91.1', response.text)
                self.assertIn(' I', response.text)
                self.assertIn(' J', response.text)
            if suffix == 'report':
                self.assertEqual(response.json()['conversion_id'], key)
            # Knowing a conversion ID never bypasses the authentication gate.
            self.assertEqual(self.client.get(route).status_code, 401)


if __name__ == '__main__':
    unittest.main()
