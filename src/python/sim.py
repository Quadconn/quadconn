import iceoryx2 as iox2

import ctypes

class JointAngles(ctypes.Structure):
    _fields_ = [
        ("hip_roll", ctypes.c_double),
        ("hip_pitch", ctypes.c_double),
        ("knee_pitch", ctypes.c_double)
    ]

    def __str__(self) -> str:
        return f"JointAngles {{ hip_roll: {self.hip_roll}, hip_pitch: {self.hip_pitch}, knee_pitch: {self.knee_pitch} }}"

    @staticmethod
    def type_name() -> str:
        # should have same val as IOX2_TYPE_NAME in cpp file
        return "JointAngles"



node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)

service = (
    node.service_builder(iox2.ServiceName.new("joint_angles"))
    .publish_subscribe(JointAngles)
    .open_or_create()
)

subscriber = service.subscriber_builder().create()

try:
    while True:
        node.wait(iox2.Duration.from_millis(500))
        while True:
            sample = subscriber.receive()
            if sample is not None:
                data = sample.payload()
                print("received: ", data.contents)

            else:
                break

except iox2.NodeWaitFailure:
    print("exit");
