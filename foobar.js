let serial = require("serial");
let storage = require("storage");
serial.setup("usart", 230400);

let PC = -2;
let boot_code = "31FEFFAF21FF9F32CB7C20FB2126FF0E113E8032E20C3EF3E2323E77773EFCE04711A8002110801ACD9500CD9600137BFE3420F311D80006081A1322230520F93E19EA1099212F990E0C3D2808320D20F92E0F18F3673E6457E0423E91E040041E020E0CF044FE9020FA0D20F71D20F20E13247C1E83FE6228061EC1FE6420067BE20C3E87E2F04290E0421520D205204F162018CB4F0604C5CB1117C1CB11170520F522232223C90000000D00091109893908C9000B0003000CCCCC000F00000000ECCCECCCDDDD99999889EEFB67636E0ECCDD1F9F8888000000000000000021A80011A8001A13BE20FE237DFE3420F506197886230520FB8620FE3E01E050";
let my_rom = "/ext/test.gb";
let file = storage.openFile(my_rom, "r", "open_existing");
let data = file.read("ascii",256);
let rom_data = data.charCodeAt(0).toString(16).toUpperCase();
let ROM = Uint8Array(256);

for (let i = 0; i < 256; i++)
{
    ROM[i] = boot_code[i];
}

while (1)
{
    PC += 2;
    file.seekRelative(PC);
    let new_data = file.read("ascii",2);
    if (new_data.length === 2)
    {
        let send = new_data.charCodeAt(0).toString(16).toUpperCase();
        print(send);
    }
    //let vgm_data = serial.readBytes(1,1000);
    //if (vgm_data === undefined) continue;
    //let data = Uint8Array(vgm_data);
    //print(data[0]);
}
