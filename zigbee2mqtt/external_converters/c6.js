import * as m from 'zigbee-herdsman-converters/lib/modernExtend';

export default {
    zigbeeModel: ['C6Fan','C6CtrlFan','ZBFanDir','ZBFanOnOff','tempSensor','Illuminance','ZBDimmable'],
    model: 'C6Fan',
    vendor: 'ihorYalosovetskyi',
    description: 'ESP32-C6 controller',
    meta: { multiEndpoint: true },
    extend: [
        m.deviceEndpoints({
            "endpoints":{
                "fan_set_speed":14,
                "fan_on_off":15,
                // "fan_pwm":10,
                "fan_dir":11,
                "fan_speed":12,
                "internal_temp":13,
                "system_init":99}
        }), 
        // m.onOff({
        //     "powerOnBehavior":false,
        //     "endpointNames":["fan_pwm"],
        //     "description": "Fan on/off",
        // }), 
        m.onOff({
            "powerOnBehavior":false,
            "endpointNames":["fan_dir"],
            "description": "Fan direction",
        }), 
        m.onOff({
            "powerOnBehavior":false,
            "endpointNames":["fan_on_off"],
            "description": "Fan on/off",
        }),         

        // m.light({
        //      endpointNames: ["fan_pwm"], // Назвемо сутність з димером 'dimmable'
        //      powerOnBehavior: false,
        //     description: "Fan speed", 
        //  }),
         
        // --- Зміни тут ---
        // Замінюємо m.light() на m.numeric() для керування швидкістю
        // m.light({
        //      endpointNames: ["fan_pwm"],
        //      effect: false,
        //      powerOnBehavior: false,
        //      description: "Керування швидкістю вентилятора",
        //      // Додаємо ці опції, щоб ігнорувати помилку READ_ONLY при записі
        //      // і не намагатися читати стан після відправки команди.
        //      configureReporting: false,
        //      readAfterWrite: false,
        //  }),
        // -----------------
        m.numeric({
            "name": "fan_set_speed",
            "unit": "%",
            valueMin: 0,
            valueMax: 100, 
            "description": "Керування швидкістю вентилятора",
            "cluster": "genAnalogOutput",
            "attribute": "presentValue",
            "endpointName": "fan_set_speed",
            "access":"STATE_SET"
        }),  
        
        m.temperature({
            "endpointNames": ["internal_temp"],
            "description": "Internal Temp",
        }),
        // m.binary({
        //     "name":"system_init",
        //     "cluster":"genBinaryInput",
        //     "attribute":"presentValue",
        //     "reporting":{"attribute":"presentValue","min":"MIN","max":"MAX","change":1},
        //     "valueOn":["ON",1],
        //     "valueOff":["OFF",0],
        //     "description":"System Initialised",
        //     "access":"STATE_GET",
        //     "endpointName":"system_init"
        // }),
        m.numeric({
            "name": "fan_speed",
            "unit": "rpm",
            valueMin: 0,
            valueMax: 5000, 
            "description": "fan rpm",
            "cluster": "genAnalogInput",
            "attribute": "presentValue",
            "reporting": {attribute: "presentValue", min: 5, max: 360000, change: 1},
            "endpointName": "fan_speed",
            "access":"STATE_GET"
        }),        
    ],    
};

