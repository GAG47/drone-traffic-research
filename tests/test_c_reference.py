"""Compile ORIGINAL C function/step blocks with a small dependency shim.

No OpenGL, no author-source edits, no reimplementation of the tested C update.
The fixture reads the pinned repository and copies selected source text unchanged.
Only types, vector accessors and CalculatePreferredVelocity are test substitutes.
MIT copyright/license attribution: THIRD_PARTY_NOTICES.md.
"""

import ctypes
from pathlib import Path
import shutil
import subprocess

import numpy as np
import pytest

from simulator import DroneState, DynamicsParameters
from simulator.dynamics import advance


def extract_function(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


@pytest.fixture(scope="module")
def c_reference(tmp_path_factory):
    compiler = shutil.which("gcc")
    if compiler is None:
        pytest.skip("GCC unavailable; analytic reference tests remain mandatory")
    root = Path(__file__).resolve().parents[1]
    robot = (root / "references/robotsim/src/robotmodel.c").read_text()
    math_source = (root / "references/robotsim/src/utilities/math_utils.c").read_text()
    force_law = extract_function(robot, "void RealCoptForceLaw(")
    unit_vector = extract_function(math_source, "void UnitVect(")
    position_block = robot[
        robot.index("/* Step coordinates (with velocity of previous TimeStepLooped) */"):
        robot.index('/* Step GPS coordinates and velocities')
    ]
    cap_block = robot[
        robot.index("double OnePerDeltaT = 1. / SitParams->DeltaT;"):
        robot.index("/* Outer Noise Term - Gaussian white noise */")
    ]
    prefix = r"""
#include <math.h>
#include <string.h>
typedef struct {
    double Coordinates[1][3], Velocities[1][3], InnerStates[1][1];
    int NumberOfInnerStates, NumberOfAgents;
} phase_t;
typedef struct { double Value; } parameter_t;
typedef struct {
    parameter_t Tau_PID_XY, Tau_PID_Z, t_GPS, t_del, a_max;
} unit_model_params_t;
typedef struct { double DeltaT; int NumberOfAgents; } sit_parameters_t;
typedef int flocking_model_params_t;
typedef int vizmode_params_t;
typedef int agent_debug_info_t;
static agent_debug_info_t DebugInfo;
static double PreferredVelocities[1][3], command[3];
void NullVect(double *v, int n) { for (int i=0; i<n; i++) v[i]=0; }
double VectAbs(double *v) { return sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]); }
void VectDifference(double *out, double *a, double *b) {
    for (int i=0; i<3; i++) out[i]=a[i]-b[i];
}
void GetAgentsVelocity(double *out, phase_t *p, int i) {
    memcpy(out,p->Velocities[i],3*sizeof(double));
}
void GetAgentsCoordinates(double *out, phase_t *p, int i) {
    memcpy(out,p->Coordinates[i],3*sizeof(double));
}
void InsertAgentsVelocity(phase_t *p, double *v, int i) {
    memcpy(p->Velocities[i],v,3*sizeof(double));
}
void InsertAgentsCoordinates(phase_t *p, double *v, int i) {
    memcpy(p->Coordinates[i],v,3*sizeof(double));
}
void CalculatePreferredVelocity(
    double *out, double *inner, phase_t *p, int i, flocking_model_params_t *f,
    vizmode_params_t *viz, double delay, double time, agent_debug_info_t *debug
) { memcpy(out,command,3*sizeof(double)); }
"""
    wrapper_start = r"""
void reference_step(double *x, double *v, double *u, double *measured,
                    double dt, double tau, double amax) {
    phase_t LocalActualPhase={0}, SteppedPhase={0}, observation={0};
    sit_parameters_t sit={dt,1}, *SitParams=&sit;
    unit_model_params_t unit={{tau},{tau},{dt},{0},{amax}}, *UnitParams=&unit;
    flocking_model_params_t f=0;
    vizmode_params_t viz=0;
    double inner[1]={0}, wind[3]={0}, Accelerations[1]={0};
    double CheckAccelerationCache[3], CheckVelocityCache[3], CheckDifferenceCache[3];
    double UnitVectDifference[3];
    int i,j,k;
    memcpy(LocalActualPhase.Coordinates[0],x,3*sizeof(double));
    memcpy(LocalActualPhase.Velocities[0],v,3*sizeof(double));
    observation=LocalActualPhase;
    memcpy(observation.Velocities[0],measured,3*sizeof(double));
    memcpy(command,u,3*sizeof(double));
"""
    force_call = r"""
    RealCoptForceLaw(SteppedPhase.Velocities[0],inner,&observation,
                    LocalActualPhase.Velocities[0],UnitParams,&f,&viz,
                    dt,0,0,0,wind);
"""
    wrapper_end = r"""
    memcpy(x,SteppedPhase.Coordinates[0],3*sizeof(double));
    memcpy(v,SteppedPhase.Velocities[0],3*sizeof(double));
}
"""
    directory = tmp_path_factory.mktemp("robotsim-c-reference")
    source = directory / "reference.c"
    library = directory / "reference.so"
    source.write_text(
        prefix + unit_vector + force_law + wrapper_start
        + position_block + force_call + cap_block + wrapper_end
    )
    completed = subprocess.run(
        [compiler, "-std=c99", "-shared", "-fPIC", "-O0", str(source), "-lm", "-o", str(library)],
        capture_output=True, text=True,
    )
    assert completed.returncode == 0, completed.stderr
    lib = ctypes.CDLL(str(library))
    vector = ctypes.POINTER(ctypes.c_double)
    lib.reference_step.argtypes = [vector] * 4 + [ctypes.c_double] * 3
    lib.reference_step.restype = None

    def step(x, v, u, measured, dt, tau, amax):
        # Run original C in its native cm units, then convert back to SI.
        arrays = [np.array([*value, 0.0], dtype=np.float64) * 100 for value in (x, v, u, measured)]
        pointers = [value.ctypes.data_as(vector) for value in arrays]
        lib.reference_step(*pointers, dt, tau, amax * 100)
        return arrays[0][:2] / 100, arrays[1][:2] / 100

    return step


@pytest.mark.parametrize("dt,tau,a_max,bias", [
    (0.01, 1.0, 6.0, [0, 0]),
    (0.05, 0.5, 2.0, [0, 0]),
    (0.1, 1.0, 100.0, [0, 0]),
    (0.01, 1.0, 6.0, [0.25, -0.1]),
])
def test_original_c_numeric_blocks_match_python(c_reference, dt, tau, a_max, bias, record_property):
    state = DroneState([17], [[2.5, -4]], [[-0.5, 0.8]], [[0, 0]])
    c_x, c_v = state.positions[0].copy(), state.velocities[0].copy()
    position_errors, velocity_errors = [], []
    for k in range(500):
        # A prescribed, discontinuously changing signal exercises acceleration,
        # deceleration, reversal and command changes without any traffic policy.
        command = ([8, 6], [-4, 3], [0, 0])[(k // 100) % 3]
        c_x, c_v = c_reference(c_x, c_v, command, c_v + bias, dt, tau, a_max)
        state, _ = advance(
            state, [command], dt, DynamicsParameters(tau, a_max),
            state.velocities + bias,
        )
        position_errors.append(np.max(np.abs(c_x - state.positions[0])))
        velocity_errors.append(np.max(np.abs(c_v - state.velocities[0])))
    max_x, max_v = float(max(position_errors)), float(max(velocity_errors))
    record_property("max_position_error_m", max_x)
    record_property("max_velocity_error_m_s", max_v)
    assert max_x < 2e-11
    assert max_v < 2e-12
