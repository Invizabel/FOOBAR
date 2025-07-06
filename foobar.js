let storage = require("storage");
let math = require("math");

function logic(my_rom)
{
    let hits = [];
    
    let file = storage.openFile(my_rom, "r", "open_existing");
    file.seekAbsolute(0x0149);
    for (let i = 0; i < 256; i++)
    {
        file.seekRelative(1);
        let raw = file.read("ascii",1);
        let data = raw.charCodeAt(0).toString(16).toUpperCase();
        
        if (data === "0")
        {
            hits.push("NOP");
        }

        if (data === "1")
        {
            let temp = "LD ";
            file.seekRelative(1);
            let raw = file.read("ascii",1);
            temp += raw.charCodeAt(0).toString() + " ";
            file.seekRelative(1);
            let raw = file.read("ascii",1);
            temp += raw.charCodeAt(0).toString();
            hits.push(temp);
        }

        if (data === "2")
        {
            hits.push("LD");
        }

        if (data === "3")
        {
            hits.push("INC");
        }

        if (data === "4")
        {
            hits.push("INC");
        }

        if (data === "5")
        {
            hits.push("DEC");
        }

        if (data === "6")
        {
            let temp = "LD ";
            file.seekRelative(1);
            let raw = file.read("ascii",1);
            temp += raw.charCodeAt(0).toString() + " ";
            hits.push(temp);
        }

        if (data === "7")
        {
            hits.push("RLCA");
        }

        if (data === "8")
        {
            let temp = "LD ";
            file.seekRelative(1);
            let raw = file.read("ascii",1);
            temp += raw.charCodeAt(0).toString() + " ";
            file.seekRelative(1);
            let raw = file.read("ascii",1);
            temp += raw.charCodeAt(0).toString();
            hits.push(temp);
        }

        if (data === "9")
        {
            hits.push("ADD");
        }
    }
    file.close();
    return hits;
}

function extract_tiles(my_rom, tile_ranges, tile_size, tile_bytes)
{
    tile_size = tile_size || 8;
    tile_bytes = tile_bytes || 16;
    let palette = [[255,255,255], [192,192,192], [96,96,96], [0,0,0]];
    let count = 0;

    storage.makeDirectory("/ext/rom_data")
    
    let file = storage.openFile(my_rom, "r", "open_existing");
    
    for (let i = 0; i < tile_ranges.length; i++)
    {
        let start = tile_ranges[i][0];
        let end = tile_ranges[i][1];
        let pos = start;

        while (pos + tile_bytes <= end + 1)
        {
            count += 1;
            file.seekAbsolute(pos);
            let raw = file.read("ascii", tile_bytes);
            let tile = [];
            let new_file = storage.openFile("/ext/rom_data/tile_" + count.toString() + ".txt", "w", "create_always");


            for (let y = 0; y < tile_size; y++)
            {
                let lo = raw.charCodeAt(2 * y);
                let hi = raw.charCodeAt(2 * y + 1);
                let row = [];

                for (let x = 0; x < tile_size; x++)
                {
                    let bit = 7 - x;
                    let l = (lo >> bit) & 1;
                    let h = (hi >> bit) & 1;
                    row.push((h << 1) | l);
                    new_file.write(((h << 1) | l).toString());
                }
                tile.push(row);
                new_file.write("\n");
            }
          
            new_file.close();
            print("Extracted tile: " + count.toString());
            pos += tile_bytes;
            
        }
    }

    file.close();
}

let my_rom = "/ext/test.gb";
let hits = logic(my_rom);
for (let i = 0; i < hits.length; i++)
{
    print(hits[i]);
}

let tile_size = 8;
let tile_bytes = 16;
let tile_tile_ranges = [[0x8000, 0x87FF], [0x8800, 0x8FFF], [0x9000, 0x97FF]];
//print("Extracting tiles.");
//extract_tiles(my_rom, tile_ranges, tile_size, tile_bytes);

//let gui = furi_record_open("gui");
//let canvas = gui_direct_draw_acquire(gui);
