"""Explicit environment-only local gain reuse; trial permission is not validation."""
import hashlib
import math
import re
from .config import ENVIRONMENT_GAIN_TRANSFER
from .system_identification import validated_support_models


def _environment(content):
    patterns={'friction':r'(?m)^MU_ROAD_CONSTANT ([+\-0-9.eE]+)$',
              'start':r'(?m)^SSTART ([+\-0-9.eE]+)$'}
    mu=re.findall(patterns['friction'],content)
    starts=re.findall(patterns['start'],content)
    if len(mu)!=1 or len(starts)!=2:
        raise ValueError('expected one friction and two start scalars')
    mu=float(mu[0]); starts=list(map(float,starts))
    cfg=ENVIRONMENT_GAIN_TRANSFER
    if not all(map(math.isfinite,[mu,*starts])) or not cfg.min_trial_friction<=mu<=cfg.max_trial_friction or starts[0]!=starts[1]:
        raise ValueError('invalid environment transfer values')
    canonical=re.sub(patterns['friction'],'MU_ROAD_CONSTANT <trial>',content)
    canonical=re.sub(patterns['start'],'SSTART <trial>',canonical)
    return canonical,mu,starts[0]


def validated_environment_models(bundle,reference,target):
    source_hash=hashlib.sha256(reference.read_bytes()).hexdigest()
    if bundle.get('source_model_sha256') != source_hash:
        raise ValueError('environment gain bundle source hash mismatch')
    models=validated_support_models(bundle,model_sha256=source_hash)
    source,source_mu,source_start=_environment(reference.read_text(encoding='utf-8'))
    destination,target_mu,target_start=_environment(target.read_text(encoding='utf-8'))
    if source!=destination:
        raise ValueError('model changed outside friction/start scalars')
    if abs(target_start-source_start)>ENVIRONMENT_GAIN_TRANSFER.max_start_station_delta_m:
        raise ValueError('start shift exceeds explicit trial range')
    return models,dict(identified_model_sha256=source_hash,
        target_model_sha256=hashlib.sha256(target.read_bytes()).hexdigest(),
        source_friction=source_mu,target_friction=target_mu,
        source_start_station_m=source_start,target_start_station_m=target_start,
        start_station_delta_m=target_start-source_start,
        kind='explicit environment-only gain reuse; not new identification',
        verification='full text identical except one MU_ROAD_CONSTANT and two equal SSTART scalars',
        scope='same vehicle/load/suspension/road geometry/interface; finite native robustness trials required')
