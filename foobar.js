let serial = require("serial");
serial.setup("usart", 230400);

while (1)
{
    let raw = serial.readBytes(1,1000);
    if (raw === undefined) continue;
    let data = Uint8Array(raw);
    print(data[0]);
}
