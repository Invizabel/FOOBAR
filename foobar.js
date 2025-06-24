let storage = require("storage");
let math = require("math");

function sum(data)
{
    let hits = 0;
    for (let i = 0; i < data.length; i++)
    {
        if (typeof data[i] === "number")
        {
            hits += data[i];
        }
    }
    return hits;
}

function mean(data)
{
    return sum(data) / data.length;
}

function extract_tiles(my_rom, ranges, tile_size, tile_bytes)
{
    tile_size = tile_size || 8;
    tile_bytes = tile_bytes || 16;
    let palette = [[255,255,255], [192,192,192], [96,96,96], [0,0,0]];
    let count = 0;

    storage.makeDirectory("/ext/temp_rom_data")
    
    let file = storage.openFile(my_rom, "r", "open_existing");
    
    for (let i = 0; i < ranges.length; i++)
    {
        let start = ranges[i][0];
        let end = ranges[i][1];
        let pos = start;

        while (pos + tile_bytes <= end + 1)
        {
            count += 1;
            file.seekAbsolute(pos);
            let raw = file.read("ascii", tile_bytes);
            let tile = [];
            let new_file = storage.openFile("/ext/temp_rom_data/tile_" + count.toString() + ".txt", "w", "create_always");


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

function convert_tiles(my_rom)
{
    storage.makeDirectory("/ext/rom_data")
    
    for (let i = 1; i <= 384; i++)
    {
        let new_tile = [];
        let temp = [];
        let file = storage.openFile("/ext/temp_rom_data/tile_" + i.toString() + ".txt", "r", "open_existing");
        for (let j = 0; j < 64; j++)
        {
            let data = file.read("ascii", 1);
            if (j % 8 === 0)
            {
                if (temp.length > 0)
                {
                    new_tile.push(temp);
                    temp = [];
                }
            }
            else
            {
                if (data === "2" || data === "3")
                {
                    temp.push(1);
                }
                if (data === "0" || data === "1")
                {
                    temp.push(0);
                }
            }
         }

        if (temp.length > 0)
        {
            new_tile.push(temp);
            temp = [];
        }

        let new_file = storage.openFile("/ext/rom_data/tile_" + i.toString() + ".txt", "w", "create_always");
        for (let row = 0; row < 8; row += 2)
        {
            for (let col = 0; col < 8; col += 2)
            {
                let tile = [new_tile[row][col], new_tile[row][col + 1], new_tile[row + 1][col], new_tile[row + 1][col + 1]];
                new_file.write(math.trunc(mean(tile)).toString());
            }
            new_file.write("\n");
        }

        print("Converted tile: " + i.toString());
    }

    storage.rmrf("/ext/temp_rom_data")
}

let tile_size = 8;
let tile_bytes = 16;
let ranges = [[0x8000, 0x87FF], [0x8800, 0x8FFF], [0x9000, 0x97FF]];
let my_rom = "/ext/kirby.gb";
print("Extracting tiles.");
extract_tiles(my_rom, ranges, tile_size, tile_bytes);
print("Converting tiles.");
convert_tiles(my_rom, ranges);

//let gui = furi_record_open("gui");
//let canvas = gui_direct_draw_acquire(gui);
