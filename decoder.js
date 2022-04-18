function dec2bin(dec) {
    var binString = (dec >>> 0).toString(2);
    for (var i = 0; i < 24-binString.length; i++) {
        binString = "0" + binString;
    }
    return binString;
  }

function getSignedInteger(bits) {
    let negative = (bits[0] === '1');
    if (negative) {
        let inverse = '';
        for (let i = 0; i < bits.length; i++) {
            inverse += (bits[i] === '0' ? '1' : '0');
        }
        return (parseInt(inverse, 2) + 1) * -1;
    } else {
        return parseInt(bits, 2);
    }
}

function decodeUplink(input) {
    var data = {};

    data.depto = input.bytes[0];
    data.finca = input.bytes[1]<<8 | input.bytes[2];
    data.vaca = input.bytes[3]<<8 | input.bytes[4];

    var latRaw = input.bytes[5]<<16 | input.bytes[6]<<8 | input.bytes[7];
    console.log(latRaw);
    data.lat = getSignedInteger(dec2bin(latRaw))/10000;
    var lngRaw = input.bytes[8]<<16 | input.bytes[9]<<8 | input.bytes[10];
    data.lng = getSignedInteger(dec2bin(lngRaw))/10000;

    console.log(data.lat)
    console.log(data.lng)

    return {
        data: data,
        warnings: [],
        errors: []
    };
}

var input = {
    bytes: [0x21, 0x01, 0xC8, 0x00, 0x7B, 0x01, 0xAD, 0xE5, 0xF4, 0x96, 0x76]
}

console.log(decodeUplink(input));