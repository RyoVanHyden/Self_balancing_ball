#ifndef SERVOKIN
#define SERVOKIN
#include <Arduino.h>
#include <stdarg.h>

namespace kine
{
    template <typename T>
    T rad2deg(T radians)
    {
        return radians * static_cast<T>(180.0 / M_PI);
    }

    template <typename T>
    T deg2rad(T degrees)
    {
        return degrees * static_cast<T>(M_PI / 180.0);
    }

    enum class coordType : uint8_t
    {
        cartesian,
        polar,
        spherical
    };

    struct cartesian
    {
        float x, y, z;
        cartesian()
        {
            x = y = z = 0;
        }

        cartesian(float x, float y, float z)
        {
            this->x = x;
            this->y = y;
            this->z = z;
        }
    };

    struct polar
    {
        float radius, phi, z;
        polar()
        {
            radius = phi = z = 0;
        }

        polar(float radius, float phi, float z)
        {
            this->radius = radius;
            this->phi = phi;
            this->z = z;
        }
    };

    struct sph
    {
        float radius, phi, theta;
        sph()
        {
            radius = phi = theta = 0;
        }

        sph(float radius, float phi, float theta)
        {
            this->radius = radius;
            this->phi = phi;
            this->theta = theta;
        }
    };

    struct servoAngles
    {
        /* data */
    };

    class points
    {
    private:
        cartesian xyz;
        polar rpz;
        sph rpt;

    public:
        points() {}

        points(float vec1, float vec2, float vec3, coordType type)
        {
            switch (type)
            {
            case coordType::cartesian:
                xyz = cartesian(vec1, vec2, vec3);
                updateFromCartesian();
                break;
            case coordType::polar:
                rpz = polar(vec1, vec2, vec3);
                updateFromPolar();
                break;
            case coordType::spherical:
                rpt = sph(vec1, vec2, vec3);
                updateFromSpherical();
                break;
            default:
                break;
            }
        }

        ~points() {}

        void updateFromCartesian()
        {
            rpz.radius = sqrt(xyz.x * xyz.x + xyz.y * xyz.y);
            rpz.phi = atan2(xyz.y, xyz.x);
            rpz.z = xyz.z;

            rpt.radius = sqrt(xyz.x * xyz.x + xyz.y * xyz.y + xyz.z * xyz.z);
            rpt.phi = atan2(xyz.y, xyz.x);
            rpt.theta = acos(xyz.z / rpt.radius);
        }

        void updateFromPolar()
        {
            xyz.x = rpz.radius * cos(rpz.phi);
            xyz.y = rpz.radius * sin(rpz.phi);
            xyz.z = rpz.z;

            rpt.radius = sqrt(rpz.radius * rpz.radius + rpz.z * rpz.z);
            rpt.phi = rpz.phi;
            rpt.theta = atan2(rpz.radius, rpz.z);
        }

        void updateFromSpherical()
        {
            xyz.x = rpt.radius * sin(rpt.theta) * cos(rpt.phi);
            xyz.y = rpt.radius * sin(rpt.theta) * sin(rpt.phi);
            xyz.z = rpt.radius * cos(rpt.theta);

            rpz.radius = rpt.radius * sin(rpt.theta);
            rpz.phi = rpt.phi;
            rpz.z = rpt.radius * cos(rpt.theta);
        }

        void setCoordinates(float vec1, float vec2, float vec3, coordType type)
        {
            switch (type)
            {
            case coordType::cartesian:
                xyz = cartesian(vec1, vec2, vec3);
                updateFromCartesian();
                break;
            case coordType::polar:
                rpz = polar(vec1, vec2, vec3);
                updateFromPolar();
                break;
            case coordType::spherical:
                rpt = sph(vec1, vec2, vec3);
                updateFromSpherical();
                break;
            default:
                break;
            }
        }

        cartesian getCartesian() const
        {
            return xyz;
        }

        polar getPolar() const
        {
            return rpz;
        }

        sph getSpherical() const
        {
            return rpt;
        }

        // Print function to display coordinates
        void printCoordinates(coordType type) const
        {
            switch (type)
            {
            case coordType::cartesian:
                Serial.print("Cartesian: (");
                Serial.print(xyz.x);
                Serial.print(", ");
                Serial.print(xyz.y);
                Serial.print(", ");
                Serial.print(xyz.z);
                Serial.println(")");
                break;

            case coordType::polar:
                Serial.print("Polar: (r=");
                Serial.print(rpz.radius);
                Serial.print(", phi=");
                Serial.print(rpz.phi);
                Serial.print(", z=");
                Serial.print(rpz.z);
                Serial.println(")");
                break;

            case coordType::spherical:
                Serial.print("Spherical: (r=");
                Serial.print(rpt.radius);
                Serial.print(", phi=");
                Serial.print(rpt.phi);
                Serial.print(", theta=");
                Serial.print(rpt.theta);
                Serial.println(")");
                break;

            default:
                break;
            }
        }

        float operator-(const points &other) const
        {
            return sqrt(pow(xyz.x - other.xyz.x, 2) + pow(xyz.y - other.xyz.y, 2) + pow(xyz.z - other.xyz.z, 2));
        }

        bool getOrientations(float *O1, float *O2, float *O3, float len1, float len2)
        {
            if (!(O1 && O2 && O3))
            {
                return false;
            }

            float r = sqrt(xyz.x * xyz.x + xyz.y * xyz.y);

            float d = sqrt(r * r + xyz.z * xyz.z);

            *O1 = kine::rad2deg(atan2(xyz.y, xyz.x));

            float phi1 = acos((len2 * len2 - len1 * len1 - d * d) / (-2 * len1 * d));
            float alpha = atan2(xyz.z, r);
            *O2 = kine::rad2deg(phi1 + alpha);

            float phi2 = acos((d * d - len1 * len1 - len2 * len2) / (-2 * len1 * len2));
            *O3 = kine::rad2deg(phi2 - M_PI);

            return true;
        }
    };

}

#endif