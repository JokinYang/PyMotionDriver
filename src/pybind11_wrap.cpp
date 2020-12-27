#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <cstdint>
#include <string>
#include <cmath>
#include <sstream>

#include "helper_3dmath.h"
//#include "MotionSensor.h"
#include "inv_mpu_lib/inv_mpu.h"
#include "inv_mpu_lib/inv_mpu_dmp_motion_driver.h"

#define wrap_180(x) (x < -180 ? x+360 : (x > 180 ? x - 360: x))
#define delay_ms(a)    usleep(a*1000)

#define YAW 0
#define PITCH 1
#define ROLL 2
#define DIM 3

// MPU control/status vars
uint8_t devStatus;      // return status after each device operation
//(0 = success, !0 = error)
uint8_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

int16_t a[3];              // [x, y, z]            accel vector
int16_t g[3];              // [x, y, z]            gyro vector
int32_t _q[4];
int32_t t;
int16_t c[3];

VectorFloat gravity;    // [x, y, z]            gravity vector

int r;
int initialized = 0;
int dmpReady = 0;
float lastval[3];
int16_t sensors;

float ypr[3];
Quaternion q;
float temp;
float gyro[3];
float accel[3];
float compass[3];

uint8_t rate = 40;

struct init_params {
    uint8_t sensors = INV_XYZ_GYRO | INV_XYZ_ACCEL;
    uint16_t gyro_fsr = 2000;
    uint16_t accel_fsr = 2;
    uint16_t dmp_feature =
            DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
    uint8_t dmp_fifo_rate = 40;
};


class Result {
public:
    Quaternion quaternion;
    VectorFloat ypr;
    VectorFloat gravity;
    VectorFloat gyro;
    VectorFloat accel;
    VectorFloat compass;
    float temp;
};

init_params params{
        INV_XYZ_GYRO | INV_XYZ_ACCEL,
        2000,
        2,
        DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL,
        40
};

uint8_t GetGravity(VectorFloat *v, Quaternion *q) {
    v->x = 2 * (q->x * q->z - q->w * q->y);
    v->y = 2 * (q->w * q->x + q->y * q->z);
    v->z = q->w * q->w - q->x * q->x - q->y * q->y + q->z * q->z;
    return 0;
}

uint8_t GetYawPitchRoll(float *data, Quaternion *q, VectorFloat *gravity) {
    // yaw: (about Z axis)
    data[0] = atan2(2 * q->x * q->y - 2 * q->w * q->z, 2 * q->w * q->w + 2 * q->x * q->x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity->x / sqrt(gravity->y * gravity->y + gravity->z * gravity->z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity->y / sqrt(gravity->x * gravity->x + gravity->z * gravity->z));
    return 0;
}


std::string setup(init_params *params) {
    dmpReady = 1;
    initialized = 0;
    for (int i = 0; i < DIM; i++) {
        lastval[i] = 10;
    }

    // initialize device
    printf("Initializing MPU...\n");
    if (mpu_init(NULL) != 0) {
        return ("MPU init failed!\n");
    }
    printf("Setting MPU sensors...\n");
//    TODO
    if (mpu_set_sensors(params->sensors) != 0) {
        return ("Failed to set sensors!");
    }
    printf("Setting GYRO sensitivity...\n");
//    TODO
    if (mpu_set_gyro_fsr(params->gyro_fsr) != 0) {
        return ("Failed to set gyro sensitivity!");
    }
    printf("Setting ACCEL sensitivity...\n");
//    TODO
    if (mpu_set_accel_fsr(params->accel_fsr) != 0) {
        return ("Failed to set accel sensitivity!");
    }
    // verify connection
    printf("Powering up MPU...\n");
    mpu_get_power_state(&devStatus);
    printf(devStatus ? "MPU6050 connection successful\n" : "MPU6050 connection failed %u\n", devStatus);

    //fifo config
    printf("Setting MPU fifo...\n");
//    TODO
    if (mpu_configure_fifo(params->sensors) != 0) {
        return ("Failed to initialize MPU fifo!");
    }

    // load and configure the DMP
    printf("Loading DMP firmware...\n");
    if (dmp_load_motion_driver_firmware() != 0) {
        return ("Failed to enable DMP!");
    }

    printf("Activating DMP...\n");
//    TODO
    if (mpu_set_dmp_state(1) != 0) {
        return ("Failed to enable DMP!");
    }

    //dmp_set_orientation()
    //if (dmp_enable_feature(DMP_FEATURE_LP_QUAT|DMP_FEATURE_SEND_RAW_GYRO)!=0) {
    printf("Configuring DMP...\n");
//    TODO
    if (dmp_enable_feature(params->dmp_feature) != 0) {
        return ("Failed to enable DMP features!");
    }

    printf("Setting DMP fifo rate...\n");
//    TODO
    if (dmp_set_fifo_rate(params->dmp_fifo_rate) != 0) {
        return ("Failed to set dmp fifo rate!");
    }
    printf("Resetting fifo queue...\n");
    if (mpu_reset_fifo() != 0) {
        return ("Failed to reset fifo!");
    }

    printf("Checking... ");
    do {
        delay_ms(1000 / rate);  //dmp will habve 4 (5-1) packets based on the fifo_rate
        r = dmp_read_fifo(g, a, _q, &sensors, &fifoCount);
    } while (r != 0 || fifoCount < 5); //packtets!!!
    printf("Done.\n");

    initialized = 1;
    return "Succeed";
};

std::string
setup_wrap(uint8_t sensors, uint16_t gyro_fsr, uint16_t accel_fsr, uint16_t dmp_feature, uint8_t dmp_fifo_rate) {
    init_params params{};
    params.sensors = sensors;
    params.gyro_fsr = gyro_fsr;
    params.accel_fsr = accel_fsr;
    params.dmp_feature = dmp_feature;
    params.dmp_fifo_rate = dmp_fifo_rate;
    return setup(&params);
}


int ms_update() {
    if (!dmpReady) {
        printf("Error: DMP not ready!!\n");
        return -1;
    }

    while (dmp_read_fifo(g, a, _q, &sensors, &fifoCount) !=
           0); //gyro and accel can be null because of being disabled in the efeatures
    q = _q;
    GetGravity(&gravity, &q);
    GetYawPitchRoll(ypr, &q, &gravity);

    mpu_get_temperature(&t);
    temp = (float) t / 65536L;

    mpu_get_compass_reg(c);

    //scaling for degrees output
    for (int i = 0; i < DIM; i++) {
        ypr[i] *= 180 / M_PI;
    }

    //unwrap yaw when it reaches 180
    ypr[0] = wrap_180(ypr[0]);

    //change sign of Pitch, MPU is attached upside down
    ypr[1] *= -1.0;

    //0=gyroX, 1=gyroY, 2=gyroZ
    //swapped to match Yaw,Pitch,Roll
    //Scaled from deg/s to get tr/s
    for (int i = 0; i < DIM; i++) {
        gyro[i] = (float) (g[DIM - i - 1]) / 131.0 / 360.0;
        accel[i] = (float) (a[DIM - i - 1]);
        compass[i] = (float) (c[DIM - i - 1]);
    }

    return 0;
}

Result *update_wrap() {
    ms_update();
    Result *result = new Result();
    result->quaternion = q;
    result->ypr = VectorFloat(ypr[0], ypr[1], ypr[2]);
    result->gravity = gravity;
    result->gyro = VectorFloat(gyro[0], gyro[1], gyro[2]);
    result->accel = VectorFloat(accel[0], accel[1], accel[2]);
    result->compass = VectorFloat(compass[0], compass[1], compass[2]);
    result->temp = temp;
    return result;
};

namespace py = pybind11;
PYBIND11_MODULE (PyMotionDriver, m) {
    m.doc() = "A python wrapper for MPU6050 gesture read";
    m.def("setup", &setup_wrap, "Setup MPU6050 with dmp feature",
          py::arg("sensors") = params.sensors,
          py::arg("gyro_fsr") = params.gyro_fsr,
          py::arg("accel_fsr") = params.accel_fsr,
          py::arg("dmp_feature") = params.dmp_feature,
          py::arg("dmp_fifo_rate") = params.dmp_fifo_rate);

    m.attr("INV_X_GYRO") = INV_X_GYRO;
    m.attr("INV_Y_GYRO") = INV_Y_GYRO;
    m.attr("INV_Z_GYRO") = INV_Z_GYRO;
    m.attr("INV_XYZ_GYRO") = INV_XYZ_GYRO;
    m.attr("INV_XYZ_ACCEL") = INV_XYZ_ACCEL;
    m.attr("INV_XYZ_COMPASS") = INV_XYZ_COMPASS;

    m.attr("DMP_FEATURE_TAP") = DMP_FEATURE_TAP;
    m.attr("DMP_FEATURE_ANDROID_ORIENT") = DMP_FEATURE_ANDROID_ORIENT;
    m.attr("DMP_FEATURE_LP_QUAT") = DMP_FEATURE_LP_QUAT;
    m.attr("DMP_FEATURE_PEDOMETER") = DMP_FEATURE_PEDOMETER;
    m.attr("DMP_FEATURE_6X_LP_QUAT") = DMP_FEATURE_6X_LP_QUAT;
    m.attr("DMP_FEATURE_GYRO_CAL") = DMP_FEATURE_GYRO_CAL;
    m.attr("DMP_FEATURE_SEND_RAW_ACCEL") = DMP_FEATURE_SEND_RAW_ACCEL;
    m.attr("DMP_FEATURE_SEND_RAW_GYRO") = DMP_FEATURE_SEND_RAW_GYRO;
    m.attr("DMP_FEATURE_SEND_CAL_GYRO") = DMP_FEATURE_SEND_CAL_GYRO;


    m.def("update", &update_wrap, py::return_value_policy::take_ownership);

    py::class_<Result>(m, "Result")
            .def_readonly("quaternion", &Result::quaternion)
            .def_readonly("ypr", &Result::ypr)
            .def_readonly("gravity", &Result::gravity)
            .def_readonly("gyro", &Result::gyro)
            .def_readonly("accel", &Result::accel)
            .def_readonly("compass", &Result::compass)
            .def_readonly("temp", &Result::temp)
            .def("__repr__", [](const Result &r) {
                std::stringstream s;
                s << "Result:\n"
                  << "YPR: \t" << r.ypr.x << "\t," << r.ypr.y << "\t," << r.ypr.z << "\n"
                  << "gravity(xyz):\t" << r.gravity.x << "\t," << r.gravity.y << "\t," << r.gravity.z << "\n"
                  << "gyro(xyz):\t" << r.gyro.x << "\t," << r.gyro.y << "\t," << r.gyro.z << "\n"
                  << "accel(xyz):\t" << r.accel.x << "\t," << r.accel.y << "\t," << r.accel.z << "\n"
                  << "compass(xyz):\t" << r.compass.x << "\t," << r.compass.y << "\t," << r.compass.z << "\n"
                  << "Temp:" << r.temp;
                return s.str();
            });

    py::class_<Quaternion>(m, "Quaternion")
            .def_readwrite("x", &Quaternion::x)
            .def_readwrite("y", &Quaternion::y)
            .def_readwrite("z", &Quaternion::z)
            .def_readwrite("w", &Quaternion::w)
            .def(py::init<>())
            .def(py::init<float, float, float, float>())
            .def("getProduct", &Quaternion::getProduct)
            .def("getConjugate", &Quaternion::getConjugate)
            .def("getMagnitude", &Quaternion::getMagnitude)
            .def("getNormalized", &Quaternion::getNormalized)
            .def("normalize", &Quaternion::normalize)
            .def("__repr__", [](const Quaternion &q) {
                std::stringstream s;
                s << "Quaternion " << "(x=" << q.x << ",y=" << q.y << ",z=" << q.z << ",w=" << q.w << ")";
                return s.str();
            });

    py::class_<VectorFloat>(m, "VectorFloat")
            .def_readwrite("x", &VectorFloat::x)
            .def_readwrite("y", &VectorFloat::y)
            .def_readwrite("z", &VectorFloat::z)
            .def(py::init<>())
            .def(py::init<float, float, float>())
            .def("normalize", &VectorFloat::normalize)
            .def("getNormalized", &VectorFloat::getNormalized)
            .def("getMagnitude", &VectorFloat::getMagnitude)
            .def("getRotated", &VectorFloat::getRotated)
            .def("rotate", &VectorFloat::rotate)
            .def("__repr__", [](const VectorFloat &vf) {
                std::stringstream s;
                s << "VectorFloat " << "(x=" << vf.x << ",y=" << vf.y << ",z=" << vf.z << ")";
                return s.str();
            });

//
//    py::class_<VectorInt16>(m, "VectorInt16")
//            .def_readwrite("x", &VectorInt16::x)
//            .def_readwrite("y", &VectorInt16::y)
//            .def_readwrite("z", &VectorInt16::z)
//            .def(py::init<>())
//            .def(py::init<int16_t, int16_t, int16_t>())
//            .def("normalize", &VectorInt16::normalize)
//            .def("getNormalized", &VectorInt16::getNormalized)
//            .def("getMagnitude", &VectorInt16::getMagnitude)
//            .def("getRotated", &VectorInt16::getRotated)
//            .def("rotate", &VectorInt16::rotate)
//            .def("__repr__", [](const VectorInt16 &vi) {
//                std::stringstream s;
//                s << "VectorInt16 " << "(x=" << vi.x << ",y=" << vi.y << ",z=" << vi.z << ")";
//                return s.str();
//            });
};
