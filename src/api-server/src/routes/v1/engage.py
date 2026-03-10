from fastapi import FastAPI, HTTPException, APIRouter
from fastapi.responses import RedirectResponse
import rclpy
from rclpy.node import Node
from roar_msgs.srv import ToggleControlSafetySwitch

router = APIRouter()

class SafetyToggleClient(Node):
    def __init__(self):
        super().__init__('safety_toggle_client')
        self.cli = self.create_client(ToggleControlSafetySwitch, '/roar/controller_manager/safety_toggle')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ToggleControlSafetySwitch.Request()

    def send_request(self, is_safety_on):
        self.req.is_safety_on = is_safety_on
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

@router.post("/toggle_safety/")
async def toggle_safety(is_safety_on: bool):
    safety_toggle_client = SafetyToggleClient()
    try:
        response = safety_toggle_client.send_request(is_safety_on)
        return {"success": True, "is_safety_on": response.status}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.on_event("startup")
async def startup_event():
    rclpy.init()
    
@router.on_event("shutdown")
def shutdown_event():
    rclpy.shutdown()
