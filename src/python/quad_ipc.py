import iceoryx2 as iox2

class QuadIpcError(Exception):
    pass

class QuadIpcSubscriber:
    def __init__(self, service_name: str, data_type: object):
        self._node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self._service = (
                self._node.service_builder(iox2.ServiceName.new(service_name))
                         .publish_subscribe(data_type)
                         .open_or_create()
        )
        self._subscriber = self._service.subscriber_builder().create()

    def wait(self, milli: int):
        try:
            self._node.wait(iox2.Duration.from_millis(milli))
        except iox2.NodeWaitFailure:
            raise QuadIpcError("Iceory Node Wait Failed")

    def receive(self):
        sample = self._subscriber.receive()
        if sample is not None:
            return sample.payload()
        return None

class QuadIpcPublisher:
    def __init__(self, service_name: str, data_type: object):
        self._node = iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)
        self._service = (
                self._node.service_builder(iox2.ServiceName.new(service_name))
                         .publish_subscribe(data_type)
                         .open_or_create()
        )
        self._subscriber = self._service.subscriber_builder().create()

    def wait(self, milli: int):
        try:
            self._node.wait(iox2.Duration.from_millis(milli))
        except iox2.NodeWaitFailure:
            raise QuadIpcError("Iceory Node Wait Failed")

    def receive(self):
        sample = self._subscriber.receive()
        if sample is not None:
            return sample.payload()
        return None