import http.client
from http.server import ThreadingHTTPServer
import json
import threading
import unittest
from backend.fallback_server import Handler
from tests.test_optimization import RIGHT
from tests.test_layer_completion import repeated_contour


class QuietHandler(Handler):
    def log_message(self,*args): pass


class ApiContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server=ThreadingHTTPServer(('127.0.0.1',0),QuietHandler)
        cls.thread=threading.Thread(target=cls.server.serve_forever,daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown(); cls.server.server_close(); cls.thread.join()

    def request(self,method,path,body=None):
        c=http.client.HTTPConnection('127.0.0.1',self.server.server_port)
        c.request(method,path,json.dumps(body) if body is not None else None,{'Content-Type':'application/json'})
        response=c.getresponse(); data=response.read(); status=response.status; c.close()
        return status,data

    def test_same_conversion_in_api_plan_and_export(self):
        status,data=self.request('POST','/api/convert',{'source':RIGHT})
        self.assertEqual(status,200)
        result=json.loads(data); key=result['id']
        self.assertEqual(key,result['report']['conversion_id'])
        status,data=self.request('GET','/api/simulate/'+key)
        self.assertEqual(status,200)
        sim=json.loads(data)
        self.assertEqual(sim['conversion_id'],key)
        self.assertEqual(sim['full_sample_count'],len(sim['samples']))
        expected=[s['sample_id'] for s in sim['samples'] if s['emit']]
        self.assertEqual(expected,[r['sample_id'] for r in sim['audit']['serialization_map'] if r['sample_id'] is not None])
        status,data=self.request('GET',f'/api/conversion/{key}/gcode')
        self.assertEqual(status,200)
        self.assertIn(key,data.decode())
        status,data=self.request('POST','/api/convert',{'source':RIGHT})
        self.assertEqual(json.loads(data)['id'],key)

    def test_bad_config_is_rejected(self):
        status,data=self.request('POST','/api/convert',{'source':RIGHT,'config':{'wrong':{}}})
        self.assertEqual(status,422)

    def test_general_curve_example_and_parametric_export(self):
        status,source=self.request('GET','/examples/contorno_organico.ngc')
        self.assertEqual(status,200)
        body={'source':source.decode('utf-8'),'config':{
            'arc_fit':{'reference_model':'PARAMETRIC_HERMITE'},
            'serialization':{'xyz_decimals':6,'c_decimals':6,'ijk_decimals':6}}}
        status,data=self.request('POST','/api/convert',body)
        self.assertEqual(status,200)
        result=json.loads(data); report=result['report']
        self.assertNotEqual(result['status'],'FAIL',report['errors'])
        self.assertTrue(report['curve_reconstruction']['applied'])
        self.assertGreater(report['native_arcs']['native_arc_length_fraction'],.85)
        self.assertLessEqual(report['validators']['NativeArcs']['max_reconstructed_export_pose_error_deg'],1.)
        status,gcode=self.request('GET',f"/api/conversion/{result['id']}/gcode")
        self.assertEqual(status,200); self.assertIn(b' I',gcode); self.assertIn(b' J',gcode)

    def test_closure_review_and_both_decisions(self):
        source=repeated_contour()
        status,data=self.request('POST','/api/files/closure-analysis',{'source':source})
        self.assertEqual(status,200)
        review=json.loads(data)
        self.assertTrue(review['review_required'])
        self.assertEqual(review['status'],'DETECTED')
        ids=[]
        for decision in ('preserve','complete'):
            body={'source':source,'config':{'layer_completion':{'decision':decision}}}
            status,data=self.request('POST','/api/convert',body)
            self.assertEqual(status,200)
            result=json.loads(data);ids.append(result['id'])
            closure=result['report']['layer_completion']
            self.assertEqual(closure['status'],'COMPLETED' if decision=='complete' else 'PRESERVED')
            self.assertEqual(closure['missing_first_layer_length_mm'],review['missing_first_layer_length_mm'])
            self.assertNotEqual(result['status'],'FAIL')
        self.assertNotEqual(*ids)
        status,data=self.request('POST','/api/files/closure-analysis',{'source':repeated_contour(closed=True)})
        self.assertEqual(status,200)
        self.assertFalse(json.loads(data)['review_required'])

    def test_parse_and_convert_share_selected_closure(self):
        body={'source':repeated_contour(),'config':{'layer_completion':{'decision':'complete'}}}
        status,data=self.request('POST','/api/files/parse',body)
        self.assertEqual(status,200)
        parsed=json.loads(data)
        status,data=self.request('POST','/api/convert',body)
        self.assertEqual(status,200)
        report=json.loads(data)['report']
        self.assertEqual(parsed['layers'],report['counts']['layers'])
        self.assertEqual(parsed['strokes'],report['counts']['strokes'])
        self.assertEqual(parsed['closure_review']['status'],'COMPLETED')


if __name__=='__main__': unittest.main()
