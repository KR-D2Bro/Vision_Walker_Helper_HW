from dbus_next.service import ServiceInterface, method
from dbus_next.aio import MessageBus
from dbus_next.constants import BusType

# D-Bus object path for the agent
AGENT_PATH = '/test/agent'

class Agent(ServiceInterface):
    def __init__(self):
        super().__init__('org.bluez.Agent1')

    @method()
    def Release(self) -> None:
        pass

    @method()
    def RequestPinCode(self, device: 'o') -> 's':
        return '0000'

    @method()
    def DisplayPinCode(self, device: 'o', pincode: 's') -> None:
        print(f'[Agent] DisplayPinCode for {device}: {pincode}')

    @method()
    def RequestPasskey(self, device: 'o') -> 'u':
        return 0

    @method()
    def DisplayPasskey(self, device: 'o', passkey: 'u', entered: 'u') -> None:
        print(f'[Agent] DisplayPasskey for {device}: {passkey} (entered {entered})')

    @method()
    def RequestConfirmation(self, device: 'o', passkey: 'u') -> None:
        # 자동 승인
        return

    @method()
    def RequestAuthorization(self, device: 'o') -> None:
        return

    @method()
    def AuthorizeService(self, device: 'o', uuid: 's') -> None:
        return

    @method()
    def Cancel(self) -> None:
        pass

async def register_agent() -> MessageBus:
    # 시스템 버스에 연결
    bus = await MessageBus(bus_type=BusType.SYSTEM).connect()
    agent = Agent()
    # 에이전트 객체를 D-Bus에 등록
    bus.export(AGENT_PATH, agent)
    # Introspection 데이터를 가져와 ProxyObject 생성
    introspection = await bus.introspect('org.bluez', '/org/bluez')
    proxy = bus.get_proxy_object(
        'org.bluez',
        '/org/bluez',
        introspection
    )
    # AgentManager1 인터페이스 얻기
    mgr = proxy.get_interface('org.bluez.AgentManager1')
    # 에이전트 등록 및 기본 에이전트 지정
    await mgr.call_register_agent(AGENT_PATH, 'NoInputNoOutput')
    await mgr.call_request_default_agent(AGENT_PATH)
    return bus

