"""Separate surface-site evidence; never changes confirmed-rock eligibility."""
from .sample_sites import SiteStore


def surface_observer(base_pose):
    """Use the actual rotation center, so pivoting is not translated evidence."""
    import numpy as np
    from .core import DRIVE_CENTER_X,rotation
    pose=list(base_pose)
    pose[:3]=(np.asarray(pose[:3])+rotation(pose[3:]) @ [DRIVE_CENTER_X,0.,0.]).tolist()
    return pose


class SoilStore(SiteStore):
    def _state(self, item, stamp):
        eligible = item['class_name']=='pale_soil_patch' and item['class_support'] >= .70
        if item['state']=='confirmed' and (item['evidence_score'] < .50 or not eligible):
            self.transition(item,'unverified',stamp,'soil identity or existence contradicted')
        if item['evidence_score'] <= .20+1e-10 and item['contradiction_count'] >= 3:
            self.transition(item,'retired',stamp,'independent visible contradictions');return
        views=item['_confirmation_views']
        if eligible and len(views)>=3 and views[-1]['stamp']-views[0]['stamp']>=2.-1e-8 and item['evidence_score']>=.90:
            self.transition(item,'confirmed',stamp,'translated multi-view ground-surface evidence')
        elif item['state'] not in ('confirmed','retired'):
            item['reason']='needs independent translated ground-surface observations'

    def _hit(self,item,obs,stamp,camera,observer):
        import numpy as np
        previous=np.asarray(item['odom_position'][:2],float)
        previous_radius=item.get('envelope_radius_m',0.)
        super()._hit(item,obs,stamp,camera,observer)
        item['last_confidence']=obs['confidence']
        if obs['geometry'].get('supported') and obs['confidence']>=.50:
            item['last_supported_stamp']=stamp
        shift=float(np.linalg.norm(previous-np.asarray(item['odom_position'][:2])))
        item['envelope_radius_m']=previous_radius+shift
        if obs['geometry'].get('supported') and obs.get('envelope_odom'):
            # A conservative circle encloses every supporting view; it never
            # shrinks to a partially visible new box during approach.
            radius=max(np.linalg.norm(np.asarray(obs['envelope_odom'])-item['odom_position'][:2],axis=1))
            item['envelope_radius_m']=max(item.get('envelope_radius_m',0.),float(radius))
