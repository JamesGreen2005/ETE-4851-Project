import unittest
from robot_explorer.telemetry_view import MotorSensors


class TelemetryViewTests(unittest.TestCase):
    def fields(self):
        return dict(E1='65530',E2='100',D1='-50',D2='-40',INA='1',
                    ST1='1',SH1='1000',BV1='24000',ST2='1',SH2='-500',BV2='23900')

    def test_both_encoder_rates_handle_rollover_and_current_units(self):
        monitor=MotorSensors();fields=self.fields();monitor.update(fields,10.)
        fields.update(E1='4',E2='80');monitor.update(fields,10.1)
        data=monitor.snapshot(fields,10.1);a,b=data['motors']
        self.assertAlmostEqual(a['counts_per_s'],100.)
        self.assertAlmostEqual(b['counts_per_s'],-200.)
        self.assertAlmostEqual(a['current_a'],.1);self.assertEqual(a['voltage_v'],24.)
        self.assertAlmostEqual(b['current_a'],-.05)

    def test_stale_missing_and_offline_do_not_show_zero_as_measurement(self):
        monitor=MotorSensors();fields=self.fields();monitor.update(fields,10.)
        data=monitor.snapshot(fields,11.)
        self.assertFalse(data['fresh'])
        self.assertIsNone(data['motors'][0]['count']);self.assertIsNone(data['motors'][0]['current_a'])
        fields['ST1']='0'
        self.assertIsNone(monitor.snapshot(fields,10.1)['motors'][0]['current_a'])
        del fields['SH2']
        self.assertIsNone(monitor.snapshot(fields,10.1)['motors'][1]['current_a'])

    def test_custom_shunt_and_invalid_resistance(self):
        monitor=MotorSensors(.05);fields=self.fields();monitor.update(fields,1.)
        self.assertAlmostEqual(monitor.snapshot(fields,1.)['motors'][0]['current_a'],.2)
        for resistance in (0.,-1.,float('nan')):
            with self.assertRaises(ValueError):MotorSensors(resistance)
