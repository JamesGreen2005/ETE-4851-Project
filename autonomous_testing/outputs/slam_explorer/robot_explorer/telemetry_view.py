"""Display conversions for existing telemetry; missing data stays unavailable."""
import math
from .core import wrap_delta


class MotorSensors:
    def __init__(self, shunt_ohms=.1):
        if not math.isfinite(shunt_ohms) or shunt_ohms<=0:
            raise ValueError('ina_shunt_ohms must be positive and finite')
        self.shunt=shunt_ohms
        self.previous=None;self.stamp=None;self.rates=[None,None]

    def update(self,fields,now):
        counts=[int(fields['E1']),int(fields['E2'])]
        dt=None if self.stamp is None else now-self.stamp
        self.rates=([wrap_delta(a,b)/dt for a,b in zip(counts,self.previous)]
                    if dt is not None and .02<=dt<=.4 else [None,None])
        self.previous=counts;self.stamp=now

    def snapshot(self,fields,now):
        age=None if self.stamp is None else max(0.,now-self.stamp)
        fresh=age is not None and age<=.4
        motors=[]
        for i in (1,2):
            item=dict(motor=i,count=None,counts_per_s=None,duty_pct=None,
                      current_a=None,voltage_v=None,power_w=None,current_status='unavailable')
            if fresh:
                item.update(count=self.previous[i-1],counts_per_s=self.rates[i-1])
                try:
                    duty=float(fields['D'+str(i)])
                    if math.isfinite(duty):item['duty_pct']=duty
                    if fields.get('INA')=='1':
                        status=int(fields['ST'+str(i)])
                        raw=int(fields['SH'+str(i)]);mv=int(fields['BV'+str(i)])
                        if status==1 and -32000<raw<32000 and 0<=mv<=32000:
                            amps=raw*.00001/self.shunt;volts=mv/1000
                            item.update(current_a=amps,voltage_v=volts,power_w=amps*volts,current_status='ok')
                        else:item['current_status']='offline or range exceeded'
                except (KeyError,ValueError,OverflowError):pass
            else:item['current_status']='stale' if age is not None else 'waiting'
            motors.append(item)
        return dict(fresh=fresh,age_s=age,motors=motors,ina_shunt_ohms=self.shunt)
