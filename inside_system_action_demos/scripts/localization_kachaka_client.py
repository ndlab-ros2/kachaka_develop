# カチャカAPI初期設定
import asyncio
import time

import kachaka_api

client = kachaka_api.aio.KachakaApiClient()

pose = await client.get_robot_pose()
print(f"{pose.x=:.2f}, {pose.y=:.2f}, {pose.theta=:.2f}")

result = await client.set_robot_pose({"x": 0.0, "y": 1.0, "theta": 0.0})
print(f"{result.success=}")

result = await client.set_robot_pose({"x": 2.0, "y": 1.0, "theta": 0.0})
print(f"{result.success=}")

# wait for requested pose to be reflected
time.sleep(5)

pose = await client.get_robot_pose()
print(f"{pose.x=:.2f}, {pose.y=:.2f}, {pose.theta=:.2f}")