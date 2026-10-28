"""Replace source startup clearance excursions with a single XY approach."""


def prepare_initial_xy_approach(motions, config):
    report = {'enabled': config.initial_xy_approach.enabled,
              'status': 'DISABLED', 'removed_motion_count': 0,
              'removed_z_motion_count': 0, 'required_start_z_mm': None,
              'selected_start_xyz': None}
    if not config.initial_xy_approach.enabled:
        return motions, report
    if not config.serialization.motion_only:
        raise ValueError('La aproximación inicial solo XY requiere salida solo movimiento; los comandos de proceso de origen no se pueden reordenar con seguridad.')
    first = next((i for i, m in enumerate(motions)
                  if m.deposition and m.kind not in ('RAPID', 'C_ONLY', 'Z_ONLY')), None)
    if first is None:
        report['status'] = 'NO_DEPOSITION'
        return motions, report
    removed = motions[:first]
    report.update(status='XY_ONLY_REQUIRES_PREPOSITIONED_Z',
                  removed_motion_count=len(removed),
                  removed_z_motion_count=sum(abs(m.end[2]-m.start[2]) > config.layers.z_change_tolerance_mm
                                             for m in removed),
                  source_start_xyz=list(motions[first].start))
    return motions[first:], report
