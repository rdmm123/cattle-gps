# LoCattle: Prototipo Tag de Bajo Consumo para Localización de Ganado Utilizando el Protocolo LoRaWAN
### Proyecto final de Ingeniería Electrónica de la Universidad del Norte.

Este es el repositorio de los sketches de Arduino utilizados para la programación y testeo del proyecto.

El código principal es `cattlegps`, el cual utiliza el método de activación OTAA para conectarse a un gateway, y enviar los datos de posición adquiridos.
El sketch `cattlegps_abp` cumple con las mismas funcionalidades que el anterior, pero utilizando el método de activación ABP.
Por otro lado, `test_with_gps_device` y `gps_basic_test` sirven para probar únicamente el sistema de localización, y permiten visualizar parámetros adicionales como el número de satelites, precisión de la posición, entre otros.
Por último, `lorawan_basic_test_otaa` sirve para probar el sistema de comunicaciones, y la conexión al gateway, enviando datos de prueba. Mientras que `lorawan_basic_test` tiene la misma funcionalidad, pero utilizando el método ABP. 
