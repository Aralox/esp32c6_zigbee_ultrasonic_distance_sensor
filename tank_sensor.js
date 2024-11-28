const {deviceEndpoints, numeric} = require('zigbee-herdsman-converters/lib/modernExtend');

const definition = {
    zigbeeModel: ['esp32c6'],
    model: 'esp32c6',
    vendor: 'ESPRESSIF',
    description: 'Ultrasonic distance sensor. Use reporting minimum interval to set sample period in seconds. Reportable change is measured in 0.01 cm.',
    extend: [
        numeric({
            name: 'Distance',
            cluster: 'msIlluminanceMeasurement',
            attribute: 'measuredValue',
            description: 'Distance',
            unit: 'cm',
            access: 'ALL',
            precision: 2,
            scale: 100,
            reporting: {min: 5, max: 0, change: 100},
            //access: 0b101
        })
    ],
    meta: {},
};

module.exports = definition;
