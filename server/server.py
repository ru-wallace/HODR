from http.server import HTTPServer, BaseHTTPRequestHandler
from gi.repository import Gio, GLib
import json
import pathlib
session_bus = Gio.bus_get_sync(Gio.BusType.SESSION, None)


proxy= Gio.DBusProxy.new_sync(session_bus, Gio.DBusProxyFlags.NONE, None,
                                'hodr.server.Control',
                                '/hodr/server/Control',
                                'hodr.server.Control', None)

script_dir = __file__.rsplit('/', 1)[0]


class RequestHandler(BaseHTTPRequestHandler):




    def do_GET(self):

        self.content_type = 'text/html'
        global proxy
        proxy= Gio.DBusProxy.new_sync(session_bus, Gio.DBusProxyFlags.NONE, None,
                                'hodr.server.Control',
                                '/hodr/server/Control',
                                'hodr.server.Control', None)
        
        


        print(f"Received request for: {self.path}")
        if self.path == '/' or self.path == '/index.html':
            self.serve_index()
        elif self.path == '/favicon.ico':
            self.content_type = 'image/x-icon'
            print("Serving favicon.ico")
            self.send_response(200)
            self.send_header('Content-type', 'image/x-icon')
            self.end_headers()
            with open(f'{script_dir}/www/favicon.ico', 'rb') as f:
                self.wfile.write(f.read())
        elif self.path == '/style.css':
            self.content_type = 'text/css'
            print("Serving style.css")
            self.send_response(200)
            self.send_header('Content-type', 'text/css')
            self.end_headers()
            with open(f'{script_dir}/www/style.css', 'rb') as f:
                self.wfile.write(f.read())
        elif self.path == '/status':
            print("Serving status")
            self.serve_status()
        elif self.path == '/temperature':
            print("Serving temperature")
            self.serve_temperature()

        elif self.path == '/target_temperature':
            print("Serving target temperature")
            self.serve_target_temperature()
        elif self.path == '/temperature_status':
            print("Serving temperature status")
            self.serve_temperature_status()
        elif self.path == '/data_ready':
            print("Checking if data is ready")
            self.serve_data_ready()
        elif self.path == '/data':
            print("Serving data")
            self.serve_data()
        elif self.path == '/number_spectra':
            print("Serving number of spectra")
            self.serve_number_spectra()
        elif self.path == '/acquisition_status':
            print("Serving acquisition status")
            self.serve_acquisition_status()
        elif self.path == '/stop_acquisition':
            print("Stopping acquisition")
            self.serve_stop_acquisition()
        elif self.path == '/power_status':
            print("Serving power status")
            self.serve_power_status()
        elif self.path == '/activate':
            print("Activating device")
            self.serve_activate()
        elif self.path == '/deactivate':
            print("Deactivating device")
            self.serve_deactivate()
        else:
            print(f"File not found: {self.path}")
            self.send_error(404, 'File Not Found: %s' % self.path)


    def serve_index(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()
        with open(f'{script_dir}/www/index.html', 'rb') as f:
            self.wfile.write(f.read()
                             )
            
    def serve_status(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        power_status = proxy.get_cached_property('active').unpack()
        power_status_str = "ON" if power_status else "OFF"
        temp = proxy.get_cached_property('Temperature').unpack()
        target_temp = proxy.get_cached_property('TargetTemperature').unpack()
        temp_status = proxy.get_cached_property('TemperatureStatus').unpack()
        n_spectra = proxy.get_cached_property('numberSpectra').unpack()
        acquisition_status = proxy.get_cached_property('acquisitionStatus').unpack()


        status = {
            'power_status': power_status_str,
            'temperature': temp,
            'target_temperature': target_temp,
            'temperature_status': temp_status,
            'number_spectra': n_spectra,
            'acquisition_status': acquisition_status

        }



        status_str = json.dumps(status, indent=4)
        
        #print(f"Status: \n{status_str}")
        self.wfile.write(status_str.encode('utf-8') + b'\n')
    
    def serve_temperature(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        temp = proxy.get_cached_property('Temperature')
        self.wfile.write(str(temp).encode('utf-8') + b'\n')
    
    def serve_target_temperature(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        target_temp = proxy.get_cached_property('TargetTemperature')
        self.wfile.write(str(target_temp).encode('utf-8') + b'\n')

    def serve_temperature_status(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        status = proxy.get_cached_property('TemperatureStatus')
        self.wfile.write(str(status).encode('utf-8') + b'\n')

    def serve_data_ready(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        data_ready = proxy.get_cached_property('dataReady')
        print(f"Data ready status: {data_ready}")
        self.wfile.write(str(data_ready).encode('utf-8') + b'\n')

    def serve_number_spectra(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        number_spectra = proxy.get_cached_property('numberSpectra')
        number_spectra = number_spectra.unpack()
        print(f"Number of spectra: {number_spectra}")
        self.wfile.write(str(number_spectra).encode('utf-8') + b'\n')
    
    def serve_acquisition_status(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        acquisition_status = proxy.get_cached_property('acquisitionStatus')
        acquisition_status = acquisition_status.unpack()
        print(f"Acquisition status: {acquisition_status}")
        self.wfile.write(str(acquisition_status).encode('utf-8') + b'\n')


    def serve_stop_acquisition(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        print("Stopping acquisition")
        proxy.call_sync('stop_acquisition', None, Gio.DBusCallFlags.NONE, -1, None)
        self.wfile.write(b'Acquisition stopped successfully\n')
        
    def serve_power_status(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        power_status = proxy.get_cached_property('active')
        power_status = power_status.unpack()
        status = ""
        if power_status:
            print("Power is ON")
            status = "ON"
        else:
            print("Power is OFF")
            status = "OFF"
        
        print(f"Power status: {power_status}")
        self.wfile.write(status.encode('utf-8') + b'\n')

    def serve_activate(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        print("Activating device")
        proxy.call_sync('activate', None, Gio.DBusCallFlags.NONE, -1, None)
        self.wfile.write(b'Device activated successfully\n')

    def serve_deactivate(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/plain')
        self.end_headers()
        print("Deactivating device")
        proxy.call_sync('deactivate', None, Gio.DBusCallFlags.NONE, -1, None)
        self.wfile.write(b'Device deactivated successfully\n')

    def serve_data(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        print("Serving data file")
        data_file = proxy.get_cached_property('dataPath')
        print(f"Data file path: {data_file}")
        data_file_str = str(data_file).strip("'").strip('"')
        data_file_str = f"../{data_file_str}"  # Ensure the path is relative to the script directory

        if not pathlib.Path(data_file_str).exists():
            print(f"Data file does not exist: {data_file_str}")
            self.send_error(404, 'Data file does not exist')
            return
        if not data_file_str:
            print("Data file path is empty")
            self.send_error(404, 'Data path is empty')
            return
        try:
            header_str = "number, timestamp, integration_time, temperature,"
            file_str = ""
            with open(data_file_str, 'r') as f:
                file_str = f.read()

            split_file_str = file_str.split('\n')[0].split(',')[4:]  # Skip the first line (header)
            for i, field in enumerate(split_file_str):
                if field.strip():
                    header_str += f"{i},"
            header_str = header_str.rstrip(',') + '\n'  # Remove trailing comma and add newline
            file_str = header_str + file_str

            print(f"Data file content: {file_str[:100]}...")  # Print first 100 characters for debugging
            self.wfile.write(file_str.encode('utf-8'))
        except FileNotFoundError:
            print(f"Data file not found: {data_file_str}")
            self.send_error(404, 'Data file not found')
        except json.JSONDecodeError:
            print("Error decoding JSON data")
            self.send_error(500, 'Error decoding JSON data')



    


    def do_POST(self):
        # Handle POST requests if needed
        if self.path == '/set_target_temperature':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            target_temp_str = post_data.decode('utf-8')
            target_temp_dict = json.loads(target_temp_str)
            target_temp = target_temp_dict.get('target_temperature')
            if target_temp is None:
                self.send_error(400, 'Missing target_temperature in request')
                return
            

            print(f"Received target temperature: {target_temp}")
            try:
                target_temp = int(target_temp)
            except ValueError:
                self.send_error(400, 'Invalid temperature value')
                return
            print(f"Setting target temperature to: {target_temp}")
            proxy.call_sync('set_temperature', GLib.Variant.new_tuple(GLib.Variant.new_int32(target_temp)), Gio.DBusCallFlags.NONE, -1, None)
            self.send_response(200)
            self.end_headers()
            self.wfile.write(b'Target temperature set successfully\n')

        elif self.path == '/start_acquisition':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            post_data_str = post_data.decode('utf-8')
            post_data_dict = json.loads(post_data_str)
            integration_time = post_data_dict.get('integration_time', -1)
            interval_time = post_data_dict.get('interval_time', -1)  # Default to -1 if not provided
            mode_str = post_data_dict.get('acquisition_mode', "")  # Default to 0 if not provided
            number = post_data_dict.get('n_captures', 1)  # Default to 1 if not provided
            print(f"Received integration time: {integration_time}")
            try:
                integration_time = float(integration_time)
            
            except ValueError:
                self.send_error(400, 'Invalid integration time value')
                return
            
            try:
                interval_time = float(interval_time)
            except ValueError:
                self.send_error(400, 'Invalid interval time value')
                return
            try:
                mode = 0
                if mode_str == 'single':
                    mode = 1
                elif mode_str == 'series':
                    mode = 3
                elif mode_str == 'continuous':
                    mode = 5
                
            except ValueError:
                self.send_error(400, 'Invalid acquisition mode value')
                return
            
            try:
                number = int(number)
            except ValueError:
                self.send_error(400, 'Invalid number of captures value')
                return
            
            print(f"Starting acquisition with:")
            print(f"Integration time: {integration_time}, Interval time: {interval_time}, Mode: {mode}, Number of captures: {number}")

            int_time_variant = GLib.Variant.new_double(integration_time)
            interval_time_variant = GLib.Variant.new_double(interval_time)
            mode_variant = GLib.Variant.new_uint32(mode)
            number_variant = GLib.Variant.new_uint32(number)

            reference = proxy.call_sync('start_acquisition', GLib.Variant.new_tuple(int_time_variant, interval_time_variant, mode_variant, number_variant), Gio.DBusCallFlags.NONE, -1, None)
            spectrum_id = reference.unpack()[0]
            self.send_response(200)
            self.end_headers()
            self.wfile.write(str(spectrum_id).encode('utf-8') + b'\n')

            return
        
        elif self.path == '/get_spectrum':
            content_length = int(self.headers['Content-Length'])
            post_data = self.rfile.read(content_length)
            post_data_str = post_data.decode('utf-8')
            print(f"Received POST data: {post_data_str}")
            post_data_dict = json.loads(post_data_str)
            spectrum_id = post_data_dict.get('spectrum_id')
            if spectrum_id is None:
                spectrum_id = -1  # Default to -1 if not provided
                
            print(f"Received spectrum ID: {spectrum_id}")
            try:
                spectrum_id = int(spectrum_id)
            except ValueError:
                self.send_error(400, 'Invalid spectrum ID value')
                return
            print(f"Retrieving spectrum with ID: {spectrum_id}")

            spectrum_id_variant = GLib.Variant.new_int32(spectrum_id)
            
            spectrum = proxy.call_sync('get_data', None, Gio.DBusCallFlags.NONE, -1, None)
            if spectrum is None:
                self.send_error(404, 'Spectrum not found')
                return
            result_tuple = spectrum.unpack()[0]

            timestamp, integration_time, temperature, data = result_tuple

            response_data = {
                'timestamp': timestamp,
                'integration_time': integration_time,
                'temperature': temperature,
                'data': data
            }
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(response_data).encode('utf-8'))
        

def main():
    server = HTTPServer(('0.0.0.0', 8080), RequestHandler)  # Replace None with your handler class
    print("Starting server on http://localhost:8080")
    server.serve_forever()

if __name__ == '__main__':
    main()
    print("Server is running on http://localhost:8080")
    print("Press Ctrl+C to stop the server.")