let serial = require("serial");
let storage = require("storage");
serial.setup("usart", 230400);

let my_rom = "/ext/test.gb";
let file = storage.openFile(my_rom, "r", "open_existing");

let index = 0;
while (index < 512)
{
    file.seekAbsolute(index*128)
    let raw = file.read("ascii",128);
    if (raw === null)
    {
        break;
    }
    let send = "";
    for (let i = 0; i < raw.length; i++)
    {
        send += raw.charCodeAt(i).toString(16);
    }
    send += "Z";
    serial.write(send);
    let recv = serial.readAny(33);
    if (recv !== undefined)
    {
        print(recv);
        index += 1;
    }
}
file.close();
