<!-- Titulo del proyecto -->
# Proyecto A4 - CrawlerBot

<!-- Logo -->
<div>
  <img src="https://github.com/tpII/2024-A4-QLEARNING-ESP32/blob/master/img/crawler.jpg">
</div>

<!-- Descripción del proyecto -->
El **Robot Crawler** se trata de un robot de estilo autito, de **ruedas no motorizadas** que se desplaza en su entorno, utilizando entonces su **brazo** de 2 grados de libertad (hombro y codo, se mueven hacia abajo y hacia arriba). 

Lo interesante es la ejecución del algoritmo de machine learning **Q-Learning**, mediante el cual el robot adquiere **inteligencia para aprender** cuales son los movimientos óptimos del brazo que generan el **desplazamiento** deseado (hacia adelante o hacia atrás).

Este proyecto tuvo como objetivo el desarrollo de uno de estos, utilizando un **ESP32**, servomotores **SG90** y encoders **HC-020K**, y desarrollando también una interfaz web de control y visualización de la ejecución del algoritmo, asegurando que se cumplan los requerimientos para que todo funcione en tiempo real.

<details>
  <summary><i>🌠Características del proyecto</i></summary>
  <ol>
    <li><b>Aprendizaje automático<b></li>
    <p>Se puede decir que es la base del proyecto. Se implementa el algoritmo Q-Learning para que el robot aprenda y ajuste sus movimientos, basándose en la recompensa calculada a partir del desplazamiento medido por los encoders.</p>
    <li>Access Point <- Servidor Web</li>
    <p>El ESP32 actúa como punto de acceso (AP) para la conexión y control remoto del robot, a través de un servidor que debemos levantar estando conectados a la Wi-Fi que este genera.</p>
    <li>Desarrollo Modular</li>
    <p>El programa está diseñado de manera modular, con componentes separados por funcionalidad, facilitando el mantenimiento y la extensión del proyecto.</p>
    <li>Replicabilidad</li>
    <p>El proyecto está documentado lo suficiente como para permitir replicar el control de un robot similar utilizando el hardware indicado.</p>
    <li>Extensibilidad</li>
    <p>Es posible ampliar el sistema para añadir mejoras de rendimiento o nuevas funcionalidades.</p>
  </ol>
    <li>Base educativa</li>
    <p>El proyecto es ideal para aprender acerca de programación de microcontroladores, conexiones hardware y conceptos avanzados de aprendizaje por refuerzo y control de robots.</p> </ol>
</details>

<details> 
  <summary><i>💻Tecnologías utilizadas</i></summary>
  <ol> 
    <li>Aplicación web</li>
    <ul> 
      <li>Django: framework de desarrollo web en Python para la construcción del servidor web.</li>
      <li>HTML, CSS y JS: fundamentales para la interfaz de usuario, ofreciendo un control intuitivo y atractivo.</li>
    </ul> 
    <li>ESP32</li>
    <ul>
      <li>Programación en C: se utilizó para implementar el control de los servos, encoders y el algoritmo de Q-learning, utilizando Visual Studio Code con la extensión PlatformIO para la gestión del firmware.</li>
      <li>Servos: el control de los servos se realizó mediante el driver `ledc` para PWM, garantizando movimientos precisos y suaves.</li>
      <li>Encoders: los encoders miden el desplazamiento del robot, retroalimentando el algoritmo de Q-learning para ajustar sus decisiones y optimizar el movimiento.</li>
      <li>Q-Learning: algoritmo implementado en C para que el robot aprenda y optimice su desplazamiento en función de las recompensas calculadas a partir de los datos de los encoders.</li>
      <li>WiFi: el ESP32 actúa como punto de acceso (AP) para la conexión y control remoto del robot, utilizando la biblioteca `esp_wifi` para configurar la red inalámbrica y manejar las solicitudes de la aplicación web.</li>
    </ul>
    <li>PlatformIO</li>
    <ul>
      <li>Entorno de desarrollo utilizado en Visual Studio Code para programar y gestionar el firmware del ESP32.</li>
    </ul> 
  </ol>
</details>

<!-- Tabla de contenidos -->
<h1 id="table-of-contents">:book: Tabla de contenidos</h1>
<details open="open">
  <summary>Tabla de contenidos</summary>
  <ol>
    <li><a href="#prerequisites-software">➤ Prerequisitos-Software</a></li>
    <li><a href="#installation-esp32">➤ Instalación y Configuración del ESP32</a></li>
    <li><a href="#installation-django-server">➤ Instalación y Configuración del Servidor Django</a></li>
    <li><a href="#execution-steps">➤ Pasos para la Ejecución del Proyecto</a></li>
    <li><a href="#video">➤ Video demostrativo</a></li>
    <li><a href="#bitacora">➤ Bitácora</a></li>
    <li><a href="#authors">➤ Autores</a></li>
    <li><a href="#coordinador">➤ Coordinador</a></li>
  </ol>
</details>



---

<!-- Prerequisitos SOFTWARE -->
<h1 id="prerequisites-software">🛠️ Prerequisitos-Software</h1>
<details>
  <summary>Prerequisitos-Software</summary>
  <p>El proyecto requiere la instalación de los siguientes componentes de software:</p>
  <ul>
    <li>
      <b>Python 3.13:</b> Lenguaje de programación para ejecutar el servidor web basado en Django. 
      Descarga desde <a href="https://www.python.org/">https://www.python.org/</a>.
    </li>
    <li>
      <b>Django:</b> Framework de desarrollo web utilizado para la implementación del servidor.
    </li>
    <li>
      <b>Visual Studio Code:</b> Editor de código necesario para manejar tanto el código en C como el servidor web.
    </li>
    <li>
      <b>PlatformIO:</b> Extensión de Visual Studio Code utilizada para compilar y cargar el firmware en el ESP32.
    </li>
  </ul>
</details>

---

<!-- Prerequisitos ESP8266 -->
<h1 id="installation-esp32">🛠️ Instalación y Configuración del ESP32</h1>
<details>
  <summary>Instalación y Configuración del ESP32</summary>
  <ol>
    <li>Abre <b>Visual Studio Code</b> y navega a la carpeta del proyecto: <code>2024-A4-QLEARNING-ESP32</code>.</li>
    <li>Posiciónate en la carpeta <code>CRAWLER-Q-LEARNING</code> (donde se encuentra el código en C).</li>
    <li>Conecta el ESP32 a un puerto USB de la computadora.</li>
    <li>Desde el menú inferior de <b>PlatformIO</b>, presiona el botón de subida de programa para compilar y cargar el firmware al ESP32.</li>
  </ol>
  <p>Tras estos pasos, el ESP32 estará configurado y listo para ejecutar las instrucciones del proyecto.</p>
</details>

---

<!-- Prerequisitos APLICACION WEB -->
<h1 id="installation-django-server">🕸️ Instalación y Configuración del Servidor Django</h1>
<details>
  <summary>Instalación y Configuración del Servidor Django</summary>
  <ol>
    <li>Posiciónate en la carpeta <code>ServidorDjango</code> dentro del proyecto.</li>
    <li>Abre una nueva terminal en Visual Studio Code.</li>
    <li>Ejecuta el siguiente comando para iniciar el servidor web:</li>
    <pre><code>python manage.py runserver 0.0.0.0:8000</code></pre>
    <li>Accede al servidor desde tu navegador en: <a href="http://localhost:8000">http://localhost:8000</a>.</li>
    <li>Asegúrate de visualizar la interfaz web del servidor correctamente.</li>
  </ol>
  <p>El servidor estará ahora listo para interactuar con el ESP32.</p>
</details>

---

<!-- Pasos para la ejecución -->
<h1 id="execution-steps">🚀 Pasos para la Ejecución del Proyecto</h1>
<details>
  <summary>Pasos para la Ejecución del Proyecto</summary>
  <ol>
    <li>Conectar el ESP32 a la computadora y cargar el firmware según los pasos en la sección <a href="#installation-esp32">Instalación y Configuración del ESP32</a>.</li>
    <li>Iniciar el servidor web siguiendo las instrucciones de la sección <a href="#installation-django-server">Instalación y Configuración del Servidor Django</a>.</li>
    <li>Desde la computadora con el servidor iniciado, conéctate a la red Wi-Fi generada por el ESP32.</li>
    <li>Accede a <a href="http://localhost:8000">http://localhost:8000</a> en tu navegador.</li>
    <li>Presiona el botón <b>Start</b> en la interfaz web para iniciar el aprendizaje del robot.</li>
    <li>Presiona el botón <b>Stop</b> para detener el aprendizaje en cualquier momento.</li>
    <li>Presiona nuevamente <b>Start</b> para ejecutar los movimientos aprendidos.</li>
    <li>Presiona <b>Stop</b> para detener el robot al finalizar la ejecución.</li>
  </ol>
</details>


<!-- video explicativo-->
<h1 id="video">🎥 Video Demostrativo </h1>
<p>A continuación, se deja a disposición un enlace a un video muy bueno acerca de la utilización del robot y su muestra funcionamiento: <a href="https://drive.google.com/file/d/1lm1mgNGavn7JolaU8XNPaSUfvHQAbiKy/view?usp=drive_link">Video</a></p>

<h1 id="bitacora">📖Bitácora</h1>

<p>Se realizó un registro de todos los avances del proyecto en la <a href="https://github.com/tpII/2024-A4-QLEARNING-ESP32/wiki/Bitacora-A4-%E2%80%90-Crawler-Robot-con-ESP32)">Bitacora</a>.</p>

<h1 id="authors">✒️ Autores</h1>

* **Ezequiel Benito Skliar** [![Repo](https://badgen.net/badge/icon/Eskliar?icon=github&label)](https://github.com/Eskliar)

* **Sebastián Sauer Rosas** [![Repo](https://badgen.net/badge/icon/sauersebastian?icon=github&label)](https://github.com/sauersebastian)
  
*  **Luciano Nicolás Loyola** [![Repo](https://badgen.net/badge/icon/LucianoLoyola?icon=github&label)](https://github.com/LucianoLoyola)


<h1 id="coordinador">📌 Coordinador</h1>

* **Alan Fabián Castelli** *Profesor - Ayudante* [![Repo](https://badgen.net/badge/icon/aCastelli95?icon=github&label)](https://github.com/aCastelli95)


<!-- Licencia -->
<h1 id="license">📄 Licencia</h1>
<details>
  <summary>Licencia</summary>
  <p>Este proyecto está bajo la Licencia <b>GPL-3.0 license</b>.</p>
  <p>Mira el archivo <code>LICENSE</code> para más detalles.</p>
</details>
