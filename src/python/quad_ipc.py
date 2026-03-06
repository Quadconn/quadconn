import iceoryx2 as iox2
import ctypes
from typing import TypeVar, Type, Optional, Any


# I have no idea if any of this works :/
T = TypeVar('T', bound=ctypes.Structure)

# --- Node & Service Builders ---

def make_node(node_name: str = "python_node") -> Any:
    """Creates and returns an iceoryx2 IPC Node."""
    # Note: Python iox2 bindings often manage the node name internally, 
    # but the creation pattern mirrors your C++ code.
    return iox2.NodeBuilder.new().create(iox2.ServiceType.Ipc)


def make_service(service_name: str, data_type: Type[T], node: Any) -> Any:
    """Creates a PublishSubscribe service for a specific ctypes structure."""
    return (
        node.service_builder(iox2.ServiceName.new(service_name))
        .publish_subscribe(data_type)
        .open_or_create()
    )


def make_subscriber(service: Any) -> Any:
    """Creates a subscriber from an existing service."""
    return service.subscriber_builder().create()


def make_publisher(service: Any) -> Any:
    """Creates a publisher from an existing service."""
    return service.publisher_builder().create()

# --- Execution & IPC Wrappers ---

def loop_waitms(ms: int, node: Any) -> bool:
    """
    Waits for a specified duration on the node.
    Safely catches the iceoryx2 Interrupt exception so Ctrl+C 
    doesn't crash the shared memory backend!
    """
    try:
        node.wait(iox2.Duration.from_millis(ms))
        return True
    except Exception: 
        # Catches KeyboardInterrupt or iox2's NodeWaitFailure
        return False


def ipc_send(data: T, publisher: Any) -> bool:
    """
    Loans uninitialized shared memory, writes the ctypes struct, and sends it.
    Returns True if sent, False if memory could not be loaned.
    """
    sample = publisher.loan_uninit()
    if sample is not None:
        sample = sample.write_payload(data)
        sample.send()
        return True
    return False


def ipc_receive(subscriber: Any) -> Optional[T]:
    """
    Safely unwraps the nested iceoryx2 receive types.
    Returns the populated ctypes payload, or None if the queue is empty.
    """
    try:
        sample = subscriber.receive()
        if sample is not None:
            # Return the populated ctypes structure directly
            return sample.payload()
            
    except Exception as e:
        print(f"[IPC Error] Failed to receive sample: {e}")
        
    return None