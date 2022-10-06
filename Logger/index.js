const { SerialPort, ReadlineParser } = require('serialport')
const fs = require('fs')






const serialport = new SerialPort({ path: 'COM2', baudRate: 9600 })
const parser = new ReadlineParser()
serialport.pipe(parser)
parser.on('data', process)


serialport.on('close', console.log)

var gauche = "const PROGMEM float moteurG_ListeVitesse[] = {"
var droite = "const PROGMEM float moteurD_ListeVitesse[] = {"

function process(data){
	
		if(data[0] == "z"){
				serialport.close()

				fs.writeFileSync("GAUCHE.txt" , gauche.slice(0,-1) + "};\n")
				
				fs.writeFileSync("DROITE.txt" , droite.slice(0,-1) + "};\n")
				return
			}
			
		var parties = data.split(",")
		gauche += (parties[0] + ",")
		droite += (parties[1].substring(0,4) + ",")
		console.log(data)
		
	}
