"""Translate the complete XY motion plan away from the nonnegative axis borders."""

from dataclasses import replace

from .gcode import motion_extent_points


def apply_xy_boundary_offset(motions, cordon_report, config):
    """Keep all layers and travels in one frame, including native arc extrema.

    The interpreter's initial (0, 0) is not a commanded point when the first
    motion is G0. Its destination is included; a first G1 starts deposition at
    that implicit origin and therefore includes it in the bounds.
    """
    settings = config.xy_boundary_offset
    points = []
    for index, motion in enumerate(motions):
        extent = motion_extent_points(motion)
        if index == 0 and motion.kind == 'RAPID':
            extent = extent[1:]
        points.extend(extent)
    if not points:
        return motions, {'enabled': settings.enabled, 'applied': False,
                         'margin_mm': settings.margin_mm, 'shift_x_mm': 0.0,
                         'shift_y_mm': 0.0, 'source_bounds_xy_mm': None,
                         'output_bounds_xy_mm': None}
    min_x = min(point[0] for point in points)
    max_x = max(point[0] for point in points)
    min_y = min(point[1] for point in points)
    max_y = max(point[1] for point in points)
    tolerance = config.geometry.zero_length_tolerance_mm
    dx = settings.margin_mm - min_x if settings.enabled and min_x <= tolerance else 0.0
    dy = settings.margin_mm - min_y if settings.enabled and min_y <= tolerance else 0.0
    report = {'enabled': settings.enabled, 'applied': bool(dx or dy),
              'margin_mm': settings.margin_mm, 'shift_x_mm': dx,
              'shift_y_mm': dy,
              'source_bounds_xy_mm': {'min_x': min_x, 'max_x': max_x,
                                      'min_y': min_y, 'max_y': max_y},
              'output_bounds_xy_mm': {'min_x': min_x + dx, 'max_x': max_x + dx,
                                      'min_y': min_y + dy, 'max_y': max_y + dy}}
    if not (dx or dy):
        return motions, report

    def shifted(point):
        return (point[0] + dx, point[1] + dy, point[2])

    translated = [replace(motion, start=shifted(motion.start),
                          end=shifted(motion.end)) for motion in motions]
    # The cordon validator compares its recorded seam to generated samples.
    # Keep the preview in source coordinates and record the output coordinates
    # separately so the operator can identify both positions.
    for field in ('actual_start_xyz', 'actual_end_xyz'):
        if cordon_report.get(field) is not None:
            cordon_report['source_' + field] = list(cordon_report[field])
            cordon_report[field] = list(shifted(cordon_report[field]))
    return translated, report
