
import utm
import numpy as np
import json
import numpy as np
import omni.physx as physx
from pxr import Gf
import Utils


class GeoJSONLoader():
    def __init__(self, utm_data, file_name,folder_path):
        self.geojson_file = f'{folder_path}/{file_name}.geojson'

        self.utm_data = utm_data
        data = self.load_json(self.geojson_file)
        self.get_data(data)

    def load_json(self, file_path):
        with open(file_path, 'r') as f:
            data = json.load(f)
        return data

    def get_data(self, geojson_data):
        self.name = geojson_data['features'][0]['properties']['Name']
        self.color = geojson_data['features'][0]['properties']['Color']
        self.elevation = geojson_data['features'][0]['properties'].get('Elevation', 10000)
        if self.elevation is None:
            self.elevation = 10000


        coordinates =np.fliplr(np.atleast_2d(geojson_data['features'][0]['geometry']['coordinates']))
        self.lat = coordinates[:,0]
        self.lon = coordinates[:,1]
        self.elevation = coordinates[:,1] * 0 + self.elevation


    def utm_to_latlon(self, utm_lat, utm_lon):
        return utm.to_latlon(utm_lat, utm_lon,  self.utm_data['zone_num'],self.utm_data['zone_letter'])


    def latlon_to_utm(self, lat, lon):
        return utm.from_latlon(lat, lon)
    

    def utm_to_enu(self, utm_lat, utm_lon):
        return np.column_stack([utm_lat - self.utm_data['lat'], utm_lon - self.utm_data['lon'], self.elevation, self.elevation*0 + 1])

    def get_collision_point(self, world,sq, xyz, max_distance=10_000.0):
            origin = Gf.Vec3f(xyz[0], xyz[1], xyz[2])
            direction = Gf.Vec3f(0.0, 0.0, -1.0) # Downward direction
            hit = sq.raycast_closest(origin, direction, max_distance)
            if hit['hit']:
                xyz[2] = hit['position'][2]
            return xyz


    def get_collisions(self, world, xyz):
        sq = physx.get_physx_scene_query_interface() # Get the PhysX scene query interface
        return [self.get_collision_point(world,sq, xyz) for xyz in xyz]
    
    def lat_lon_to_enu(self):
        utm_coords = self.latlon_to_utm(self.lat, self.lon)
        return self.utm_to_enu(utm_coords[0], utm_coords[1])


    def get_spline(self):
        coords_enu = self.lat_lon_to_enu()
        spline_points, spline_points_der,euler_initial_angles = Utils.generate_spline_path_from_enu(coords_enu, spline_param = 3, num_samples = 1000, add_z = 0)
        return spline_points, spline_points_der, euler_initial_angles





