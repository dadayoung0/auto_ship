import asyncio


class Test:
    def __init__(self):
        self.is_detected = False

    async def object_d(self):
        self.is_detected = True
        await asyncio.sleep(2)
        print('객체탐지 완료')
        self.is_detected = False

    async def run(self):
        asyncio.create_task(self.object_d())
        for i in range(10000000):
            if not self.is_detected:
                asyncio.create_task(self.object_d())
            await asyncio.sleep(0.1)
            print(i, '모터 동작')


t = Test()
asyncio.run(t.run())
