from dataclasses import dataclass, field, asdict


@dataclass
class Geometry:
    mode: str = "SMOOTH_CORNERS"
    max_xy_deviation_mm: float = 1e-6
    zero_length_tolerance_mm: float = 1e-6
    tiny_segment_warning_mm: float = 0.05


@dataclass
class Layers:
    z_change_tolerance_mm: float = 0.001


@dataclass
class Orientation:
    mode: str = "perpendicular"
    perpendicular_sign: int = 1
    alignment_offset_deg: float = 90.0
    calibration_offset_deg: float = 0.0
    symmetry_period_deg: float = 360.0
    preorient_during_travel: bool = False
    # Explicit startup reference. ROTATE_AT_START lets the operator home C=0
    # along a repeatable machine axis and performs the required C-only move at
    # the first printable point.  The boolean remains as an input-compatibility
    # alias for profiles created before 3.7.0.
    initial_orientation_mode: str = "ROTATE_AT_START"
    home_axis_deg: float = 0.0
    initial_rotation_speed_deg_s: float = 10.0
    home_aligned_start: bool = False


@dataclass
class CornerDetection:
    smooth_turn_max_deg: float = 8.0
    sharp_corner_min_deg: float = 45.0
    curvature_relative_tolerance: float = 0.25


@dataclass
class CornerPlanner:
    strategy: str = "adaptive"
    max_pretransition_mm: float = 20.0
    max_fraction_of_incoming_segment: float = 0.60
    minimum_transition_mm: float = 1.0
    allow_c_only_at_sharp_corner: bool = False
    fallback: str = "reduce_feed_recommendation"


@dataclass
class CornerSmoothing:
    enabled: bool = True
    detection_threshold_deg: float = 25.0
    radius_mode: str = "adaptive"
    radius_mm: float = 10.0
    max_path_deviation_mm: float | None = None
    min_segment_remaining_mm: float = 5.0
    max_delta_c_deg: float = 15.0
    max_chord_error_mm: float = 0.30
    preferred_radius_nozzle_factor: float = 1.0
    max_radius_mm: float | None = None


@dataclass
class Sampling:
    mode: str = 'adaptive_global_curves'
    max_delta_c_deg: float = 15.0
    max_orientation_interp_error_deg: float = 1.0
    max_linear_step_mm: float = 10.0
    geometry_error_mm: float = 0.30
    preferred_min_block_length_mm: float = 2.0
    max_samples_per_stroke: int = 100000


@dataclass
class Simplification:
    enabled: bool = True
    geometry_tolerance_mm: float = 0.30
    orientation_tolerance_deg: float = 1.0
    collinear_angle_tolerance_deg: float = 0.05


@dataclass
class Nozzle:
    length_mm: float = 35.0
    width_mm: float = 15.0


@dataclass
class ZConversion:
    enabled: bool = True
    mode: str = 'normalized_layer_schedule'
    first_layer_z_mm: float = 20.0
    subsequent_layer_height_mm: float = 18.0
    preserve_first_layer_if_already_target: bool = True
    z_match_tolerance_mm: float = 0.10
    apply_to_deposition_layers: bool = True
    apply_to_layer_transitions: bool = True
    preserve_explicit_machine_safe_travel_z: bool = True


@dataclass
class LinuxCNC:
    path_mode: str = 'PRESERVE_SOURCE'
    blend_tolerance_mm: float | None = None


@dataclass
class LayerTransition:
    safe_seam: bool = False
    corner_guard_mm: float = 20.0
    alternate_open_paths: bool = False


@dataclass
class LayerCompletion:
    decision: str = 'preserve'
    match_tolerance_mm: float = 0.001
    min_reference_layers: int = 2


@dataclass
class Cordon:
    mode: str = 'legacy'
    length_mm: float = 65.0


@dataclass
class Process:
    feed_policy: str = 'PRESERVE_SOURCE'
    experimental_feed_factor: float = 1.0


@dataclass
class Precision:
    geometry_epsilon: float = 1e-9
    angle_epsilon_deg: float = 1e-8


@dataclass
class Kinematics:
    max_c_velocity_deg_s: float | None = None
    max_c_acceleration_deg_s2: float | None = None


@dataclass
class Serialization:
    xyz_decimals: int = 3
    c_decimals: int = 3
    feed_decimals: int = 3
    preserve_comments: bool = True
    preserve_unknown_commands: bool = True
    motion_only: bool = True
    ijk_decimals: int = 4
    radius_decimals: int = 4
    motion_mode: str = 'HYBRID_NATIVE_CURVES'


@dataclass
class SourceArcPolicy:
    preserve: bool = True
    format: str = 'CENTER_IJK'


@dataclass
class CornerArcPolicy:
    enabled: bool = True
    format: str = 'RADIUS_R'


@dataclass
class FittedArcPolicy:
    enabled: bool = True
    format: str = 'CENTER_IJK'


@dataclass
class NativeArcs:
    enabled: bool = True
    radius_format_max_sweep_deg: float = 165.0
    auto_fallback_to_ijk: bool = True
    source_arcs: SourceArcPolicy = field(default_factory=SourceArcPolicy)
    corner_fillets: CornerArcPolicy = field(default_factory=CornerArcPolicy)
    fitted_free_curves: FittedArcPolicy = field(default_factory=FittedArcPolicy)


@dataclass
class ArcOutput:
    format: str = 'AUTO'


@dataclass
class ArcFit:
    enabled: bool = True
    biarc_enabled: bool = True
    geometry_tolerance_mm: float = 0.30
    radial_tolerance_mm: float = 0.30
    tangent_tolerance_deg: float = 1.0
    orientation_tolerance_deg: float = 1.0
    min_radius_mm: float = 5.0
    max_radius_mm: float | None = None
    max_sweep_deg: float = 160.0
    split_at_inflections: bool = True
    minimum_source_points: int = 3
    adaptive_refinement: bool = True
    max_refinement_depth: int = 2
    reference_model: str = 'POLYLINE_C'
    reconstruction_budget_fraction: float = 0.5


@dataclass
class ExperimentalSplines:
    enabled: bool = False


@dataclass
class Config:
    native_arcs: NativeArcs = field(default_factory=NativeArcs)
    arc_output: ArcOutput = field(default_factory=ArcOutput)
    arc_fit: ArcFit = field(default_factory=ArcFit)
    experimental_splines: ExperimentalSplines = field(default_factory=ExperimentalSplines)
    nozzle: Nozzle = field(default_factory=Nozzle)
    z_conversion: ZConversion = field(default_factory=ZConversion)
    linuxcnc: LinuxCNC = field(default_factory=LinuxCNC)
    geometry: Geometry = field(default_factory=Geometry)
    layers: Layers = field(default_factory=Layers)
    orientation: Orientation = field(default_factory=Orientation)
    corner_detection: CornerDetection = field(default_factory=CornerDetection)
    corner_planner: CornerPlanner = field(default_factory=CornerPlanner)
    corner_smoothing: CornerSmoothing = field(default_factory=CornerSmoothing)
    sampling: Sampling = field(default_factory=Sampling)
    kinematics: Kinematics = field(default_factory=Kinematics)
    serialization: Serialization = field(default_factory=Serialization)
    simplification: Simplification = field(default_factory=Simplification)
    layer_transition: LayerTransition = field(default_factory=LayerTransition)
    layer_completion: LayerCompletion = field(default_factory=LayerCompletion)
    cordon: Cordon = field(default_factory=Cordon)
    process: Process = field(default_factory=Process)
    precision: Precision = field(default_factory=Precision)

    @classmethod
    def from_dict(cls, data: dict | None):
        data = data or {}
        result = cls()
        if not isinstance(data, dict) or set(data) - set(result.__dataclass_fields__):
            raise ValueError('Sección de configuración desconocida')
        for name in result.__dataclass_fields__:
            if name in data:
                if not isinstance(data[name], dict):
                    raise ValueError(f'{name} debe ser un objeto')
                section = getattr(result, name)
                def assign(section,values,prefix):
                    if not isinstance(values,dict): raise ValueError(f'{prefix} debe ser un objeto')
                    for key,value in values.items():
                        if key not in section.__dataclass_fields__: raise ValueError(f'Parámetro desconocido: {prefix}.{key}')
                        current=getattr(section,key)
                        if hasattr(current,'__dataclass_fields__'): assign(current,value,prefix+'.'+key)
                        else: setattr(section,key,value)
                assign(section,data[name],name)
        # Profiles from 3.6 and earlier only contained home_aligned_start.
        # Preserve their exact meaning unless the new mode is explicit.
        orientation_data = data.get('orientation', {}) if isinstance(data, dict) else {}
        if ('home_aligned_start' in orientation_data and
                'initial_orientation_mode' not in orientation_data):
            result.orientation.initial_orientation_mode = (
                'HOME_ALIGNED_TANGENT' if result.orientation.home_aligned_start
                else 'ABSOLUTE_MODEL')
        elif 'initial_orientation_mode' in orientation_data:
            result.orientation.home_aligned_start = (
                result.orientation.initial_orientation_mode == 'HOME_ALIGNED_TANGENT')
        result.validate()
        return result

    def validate(self):
        import math
        for section, values in self.to_dict().items():
            for key, value in values.items():
                if isinstance(value, (int, float)) and not math.isfinite(value):
                    raise ValueError(f'{section}.{key} debe ser finito')
        positive = [("orientation.symmetry_period_deg", self.orientation.symmetry_period_deg),
                    ("sampling.max_delta_c_deg", self.sampling.max_delta_c_deg),
                    ("sampling.max_orientation_interp_error_deg", self.sampling.max_orientation_interp_error_deg),
                    ("sampling.max_linear_step_mm", self.sampling.max_linear_step_mm),
                    ("corner_planner.max_pretransition_mm", self.corner_planner.max_pretransition_mm),
                    ("corner_planner.minimum_transition_mm", self.corner_planner.minimum_transition_mm)]
        positive.extend([("corner_smoothing.radius_mm", self.corner_smoothing.radius_mm),
                         ("corner_smoothing.max_delta_c_deg", self.corner_smoothing.max_delta_c_deg),
                         ("corner_smoothing.max_chord_error_mm", self.corner_smoothing.max_chord_error_mm)])
        positive.extend([(name, value) for name, value in (
            ('geometry.max_xy_deviation_mm', self.geometry.max_xy_deviation_mm),
            ('geometry.zero_length_tolerance_mm', self.geometry.zero_length_tolerance_mm),
            ('simplification.geometry_tolerance_mm', self.simplification.geometry_tolerance_mm),
            ('simplification.orientation_tolerance_deg', self.simplification.orientation_tolerance_deg),
            ('precision.geometry_epsilon', self.precision.geometry_epsilon),
            ('precision.angle_epsilon_deg', self.precision.angle_epsilon_deg),
            ('layer_completion.match_tolerance_mm', self.layer_completion.match_tolerance_mm),
            ('cordon.length_mm', self.cordon.length_mm),
            ('process.experimental_feed_factor', self.process.experimental_feed_factor))])
        positive.extend([('nozzle.length_mm',self.nozzle.length_mm),
                         ('nozzle.width_mm',self.nozzle.width_mm),
                         ('corner_smoothing.preferred_radius_nozzle_factor',self.corner_smoothing.preferred_radius_nozzle_factor),
                         ('sampling.geometry_error_mm',self.sampling.geometry_error_mm),
                         ('z_conversion.subsequent_layer_height_mm',self.z_conversion.subsequent_layer_height_mm)])
        for name,value in [('corner_smoothing.max_path_deviation_mm',self.corner_smoothing.max_path_deviation_mm),
                           ('corner_smoothing.max_radius_mm',self.corner_smoothing.max_radius_mm),
                           ('linuxcnc.blend_tolerance_mm',self.linuxcnc.blend_tolerance_mm)]:
            if value is not None: positive.append((name,value))
        for name, value in positive:
            if not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} debe ser positivo y finito")
        for key in ('geometry_tolerance_mm','radial_tolerance_mm','tangent_tolerance_deg',
                    'orientation_tolerance_deg','min_radius_mm','max_sweep_deg'):
            if getattr(self.arc_fit,key) <= 0: raise ValueError(f'arc_fit.{key} debe ser positivo')
        if not 0 < self.native_arcs.radius_format_max_sweep_deg < 180:
            raise ValueError('radius_format_max_sweep_deg debe estar entre 0 y 180')
        if self.arc_output.format not in ('AUTO','RADIUS_R','CENTER_IJK'):
            raise ValueError('Formato de arco inválido')
        for policy,flag in ((self.native_arcs.source_arcs,'preserve'),(self.native_arcs.corner_fillets,'enabled'),
                            (self.native_arcs.fitted_free_curves,'enabled')):
            if type(getattr(policy,flag)) is not bool or policy.format not in ('AUTO','RADIUS_R','CENTER_IJK'):
                raise ValueError('Política de arcos nativos inválida')
        if self.serialization.motion_mode not in ('HYBRID_NATIVE_CURVES','G1_ONLY'):
            raise ValueError('motion_mode inválido')
        if self.arc_fit.max_sweep_deg > 180 or self.arc_fit.minimum_source_points < 3:
            raise ValueError('Barrido o cantidad de puntos de ajuste inválidos')
        if type(self.arc_fit.minimum_source_points) is not int:
            raise ValueError('minimum_source_points debe ser entero')
        if type(self.arc_fit.max_refinement_depth) is not int or not 0<=self.arc_fit.max_refinement_depth<=5:
            raise ValueError('max_refinement_depth debe ser entero entre 0 y 5')
        if self.arc_fit.reference_model not in ('POLYLINE_C','PARAMETRIC_HERMITE'):
            raise ValueError('reference_model debe ser POLYLINE_C o PARAMETRIC_HERMITE')
        if (type(self.arc_fit.reconstruction_budget_fraction) not in (int,float) or
                not 0<self.arc_fit.reconstruction_budget_fraction<1):
            raise ValueError('reconstruction_budget_fraction debe estar entre 0 y 1, sin extremos')
        if self.arc_fit.max_radius_mm is not None and self.arc_fit.max_radius_mm < self.arc_fit.min_radius_mm:
            raise ValueError('max_radius_mm menor que min_radius_mm')
        for section, keys in ((self.native_arcs,('enabled','auto_fallback_to_ijk')),
                              (self.arc_fit,('enabled','biarc_enabled','split_at_inflections','adaptive_refinement'))):
            if any(type(getattr(section,k)) is not bool for k in keys): raise ValueError('Flags de arcos deben ser booleanos')
        if self.experimental_splines.enabled or not self.arc_fit.split_at_inflections:
            raise ValueError('Splines experimentales o ajuste atravesando inflexiones no habilitados')
        if not 0 < self.corner_planner.max_fraction_of_incoming_segment <= 1:
            raise ValueError("max_fraction_of_incoming_segment debe estar entre 0 y 1")
        if not 0 <= self.corner_detection.smooth_turn_max_deg < self.corner_detection.sharp_corner_min_deg <= 180:
            raise ValueError("Umbrales de esquina inválidos")
        if self.corner_planner.strategy not in ("adaptive", "preturn_blend", "exact_vertex"):
            raise ValueError("Estrategia de esquina inválida")
        if self.geometry.mode not in ("PRESERVE_PATH", "SMOOTH_CORNERS"):
            raise ValueError("Modo de geometría inválido")
        if self.corner_smoothing.radius_mode not in ('fixed','adaptive'):
            raise ValueError('radius_mode debe ser fixed o adaptive')
        if self.sampling.mode not in ('adaptive_global_curves','legacy'):
            raise ValueError('Modo de sampling inválido')
        if self.z_conversion.mode != 'normalized_layer_schedule':
            raise ValueError('Modo Z inválido')
        if self.z_conversion.z_match_tolerance_mm < 0 or self.sampling.preferred_min_block_length_mm < 0:
            raise ValueError('Tolerancias no negativas requeridas')
        if self.linuxcnc.path_mode not in ('PRESERVE_SOURCE','G64'):
            raise ValueError('Modo LinuxCNC inválido')
        if not isinstance(self.serialization.motion_only, bool):
            raise ValueError('serialization.motion_only debe ser booleano')
        if not 0 < self.corner_smoothing.detection_threshold_deg < 180:
            raise ValueError("Umbral de fillet inválido")
        if not 0 <= self.corner_smoothing.min_segment_remaining_mm:
            raise ValueError("min_segment_remaining_mm inválido")
        if self.orientation.preorient_during_travel:
            raise ValueError("La preorientación durante travel todavía no está implementada")
        if type(self.orientation.home_aligned_start) is not bool:
            raise ValueError('orientation.home_aligned_start debe ser booleano')
        if self.orientation.initial_orientation_mode not in (
                'ROTATE_AT_START', 'HOME_ALIGNED_TANGENT', 'ABSOLUTE_MODEL'):
            raise ValueError('orientation.initial_orientation_mode inválido')
        if self.orientation.home_axis_deg not in (-90, 0, 90, 180):
            raise ValueError('orientation.home_axis_deg debe ser -90, 0, 90 o 180')
        if (not isinstance(self.orientation.initial_rotation_speed_deg_s, (int, float)) or
                not math.isfinite(self.orientation.initial_rotation_speed_deg_s) or
                self.orientation.initial_rotation_speed_deg_s <= 0):
            raise ValueError('orientation.initial_rotation_speed_deg_s debe ser positivo y finito')
        if self.orientation.mode not in ('perpendicular', 'alignment'):
            raise ValueError('orientation.mode debe ser perpendicular o alignment')
        if self.orientation.perpendicular_sign not in (-1, 1):
            raise ValueError('perpendicular_sign debe ser -1 o +1')
        if self.orientation.symmetry_period_deg not in (180, 360):
            raise ValueError('symmetry_period_deg debe ser 180 o 360')
        if self.process.feed_policy not in ('PRESERVE_SOURCE', 'RECOMMEND', 'ADAPTIVE_EXPERIMENTAL'):
            raise ValueError('Política de feed inválida')
        if self.layer_completion.decision not in ('preserve', 'complete'):
            raise ValueError('layer_completion.decision debe ser preserve o complete')
        if self.cordon.mode not in ('legacy', 'add', 'keep', 'omit'):
            raise ValueError('cordon.mode debe ser legacy, add, keep u omit')
        if not 60 <= self.cordon.length_mm <= 70:
            raise ValueError('cordon.length_mm debe estar entre 60 y 70 mm')
        if (type(self.layer_completion.min_reference_layers) is not int or
                not 2 <= self.layer_completion.min_reference_layers <= 1000):
            raise ValueError('min_reference_layers debe ser un entero entre 2 y 1000')
        if self.layer_transition.corner_guard_mm < 0 or not 0 <= self.corner_detection.curvature_relative_tolerance <= 1:
            raise ValueError('Guard o tolerancia de curvatura inválida')
        if type(self.layer_transition.safe_seam) is not bool or type(self.layer_transition.alternate_open_paths) is not bool:
            raise ValueError('Las políticas de cambio de capa deben ser booleanas')
        for name in ("max_c_velocity_deg_s", "max_c_acceleration_deg_s2"):
            value = getattr(self.kinematics, name)
            if value is not None and (not math.isfinite(value) or value <= 0):
                raise ValueError(f"{name} debe ser positivo")
        if not 1 <= self.sampling.max_samples_per_stroke <= 2000000:
            raise ValueError("max_samples_per_stroke fuera de rango")
        for name in ("xyz_decimals", "c_decimals", "feed_decimals", "ijk_decimals", "radius_decimals"):
            value = getattr(self.serialization, name)
            if type(value) is not int or not 0 <= value <= 9:
                raise ValueError(f"{name} fuera de rango")

    def to_dict(self):
        return asdict(self)

    @classmethod
    def legacy(cls):
        """Explicit reproducible comparison profile; never the production default."""
        return cls.from_dict({'native_arcs':{'enabled':False},'arc_fit':{'enabled':False},'sampling':{'mode':'legacy','max_delta_c_deg':2.,
                            'max_linear_step_mm':5.,'max_orientation_interp_error_deg':.25},
                             'corner_smoothing':{'radius_mode':'fixed','radius_mm':10.,'detection_threshold_deg':30.,
                            'max_path_deviation_mm':5.,'min_segment_remaining_mm':2.,
                            'max_delta_c_deg':2.,'max_chord_error_mm':.1},
                             'simplification':{'geometry_tolerance_mm':.03,'orientation_tolerance_deg':.1},
                             'orientation':{'initial_orientation_mode':'HOME_ALIGNED_TANGENT'},
                             'z_conversion':{'enabled':False}})

    @property
    def orientation_offset_deg(self):
        # Keep the existing alignment control; sign applies only to perpendicular alignment.
        alignment = self.orientation.alignment_offset_deg
        if self.orientation.mode == 'perpendicular' and abs(alignment) == 90:
            alignment = 90 * self.orientation.perpendicular_sign
        return alignment + self.orientation.calibration_offset_deg
