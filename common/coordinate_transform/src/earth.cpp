#include "coordinate_transform/earth.hpp"

namespace earth
{
    double loc_x, loc_y, loc_z;

    void geo2loc(double lat, double lon, double ell)
    {
        double PS, PSo, PDL, Pt, PN, PW;
        double PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9;
        double PA, PB, PC, PD, PE, PF, PG, PH, PI;
        double Pe, Pet, Pnn, AW, FW, Pmo;

        double m_lat = lat / 180.0 * M_PI;
        double m_lon = lon / 180.0 * M_PI;
        double m_PLo = 121.0 / 180.0 * M_PI;
        double m_PLato = 0.0 / 180.0 * M_PI;

        // WGS84 Parameters
        AW = 6378137.0;
        FW = 1.0 / 298.257222101;
        Pmo = 0.9999;

        Pe = static_cast<double>(std::sqrt(2.0 * FW - std::pow(FW, 2)));
        Pet = static_cast<double>(std::sqrt(std::pow(Pe, 2) / (1.0 - std::pow(Pe, 2))));

        PA = static_cast<double>(1.0 + 3.0 / 4.0 * std::pow(Pe, 2) + 45.0 / 64.0 * std::pow(Pe, 4) +
          175.0 / 256.0 * std::pow(Pe, 6) + 11025.0 / 16384.0 * std::pow(Pe, 8) + 43659.0 / 65536.0 *
          std::pow(Pe, 10) + 693693.0 / 1048576.0 * std::pow(Pe, 12) + 19324305.0 / 29360128.0 * std::pow(Pe, 14) +
          4927697775.0 / 7516192768.0 * std::pow(Pe, 16));

        PB = static_cast<double>(3.0 / 4.0 * std::pow(Pe, 2) + 15.0 / 16.0 * std::pow(Pe, 4) + 525.0 / 512.0 *
          std::pow(Pe, 6) + 2205.0 / 2048.0 * std::pow(Pe, 8) + 72765.0 / 65536.0 * std::pow(Pe, 10) + 297297.0 / 262144.0 *
          std::pow(Pe, 12) + 135270135.0 / 117440512.0 * std::pow(Pe, 14) + 547521975.0 / 469762048.0 * std::pow(Pe, 16));

        PC = static_cast<double>(15.0 / 64.0 * std::pow(Pe, 4) + 105.0 / 256.0 * std::pow(Pe, 6) + 2205.0 / 4096.0 *
          std::pow(Pe, 8) + 10395.0 / 16384.0 * std::pow(Pe, 10) + 1486485.0 / 2097152.0 * std::pow(Pe, 12) +
          45090045.0 / 58720256.0 * std::pow(Pe, 14) + 766530765.0 / 939524096.0 * std::pow(Pe, 16));

        PD = static_cast<double>(35.0 / 512.0 * std::pow(Pe, 6) + 315.0 / 2048.0 * std::pow(Pe, 8) + 31185.0 / 131072.0 *
            std::pow(Pe, 10) + 165165.0 / 524288.0 * std::pow(Pe, 12) + 45090045.0 / 117440512.0 * std::pow(Pe, 14) +
            209053845.0 / 469762048.0 * std::pow(Pe, 16));
        
        PE = static_cast<double>(315.0 / 16384.0 * std::pow(Pe, 8) + 3465.0 / 65536.0 * std::pow(Pe, 10) +
            99099.0 / 1048576.0 * std::pow(Pe, 12) + 4099095.0 / 29360128.0 * std::pow(Pe, 14) + 348423075.0 / 1879048192.0 *
            std::pow(Pe, 16));
        
        PF = static_cast<double>(693.0 / 131072.0 * std::pow(Pe, 10) + 9009.0 / 524288.0 * std::pow(Pe, 12) +
            4099095.0 / 117440512.0 * std::pow(Pe, 14) + 26801775.0 / 469762048.0 * std::pow(Pe, 16));
        
        PG = static_cast<double>(3003.0 / 2097152.0 * std::pow(Pe, 12) + 315315.0 / 58720256.0 * std::pow(Pe, 14) +
            11486475.0 / 939524096.0 * std::pow(Pe, 16));
        
        PH = static_cast<double>(45045.0 / 117440512.0 * std::pow(Pe, 14) + 765765.0 / 469762048.0 * std::pow(Pe, 16));
        
        PI = static_cast<double>(765765.0 / 7516192768.0 * std::pow(Pe, 16));
        
        PB1 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PA;
        PB2 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PB / -2.0;
        PB3 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PC / 4.0;
        PB4 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PD / -6.0;
        PB5 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PE / 8.0;
        PB6 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PF / -10.0;
        PB7 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PG / 12.0;
        PB8 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PH / -14.0;
        PB9 = static_cast<double>(AW) * (1.0 - std::pow(Pe, 2)) * PI / 16.0;
        
        PS = static_cast<double>(PB1) * m_lat + PB2 * std::sin(2.0 * m_lat) + PB3 * std::sin(4.0 * m_lat) + PB4 * std::sin(6.0 * m_lat) + PB5 * std::sin(8.0 * m_lat) + PB6 * std::sin(10.0 * m_lat) + PB7 * std::sin(12.0 * m_lat) + PB8 * std::sin(14.0 * m_lat) + PB9 * std::sin(16.0 * m_lat);
        
        PSo = static_cast<double>(PB1) * m_PLato + PB2 * std::sin(2.0 * m_PLato) + PB3 * std::sin(4.0 * m_PLato) + PB4 * std::sin(6.0 * m_PLato) + PB5 * std::sin(8.0 * m_PLato) + PB6 * std::sin(10.0 * m_PLato) + PB7 * std::sin(12.0 * m_PLato) + PB8 * std::sin(14.0 * m_PLato) + PB9 * std::sin(16.0 * m_PLato);
        
        PDL = static_cast<double>(m_lon) - m_PLo;
        Pt = static_cast<double>(std::tan(m_lat));
        PW = static_cast<double>(std::sqrt(1.0 - std::pow(Pe, 2) * std::pow(std::sin(m_lat), 2)));
        PN = static_cast<double>(AW) / PW;
        Pnn = static_cast<double>(std::sqrt(std::pow(Pet, 2) * std::pow(std::cos(m_lat), 2)));
        
        loc_y = static_cast<double>(((PS - PSo) + (1.0 / 2.0) * PN * std::pow(std::cos(m_lat), 2.0) * Pt * std::pow(PDL, 2.0) +
            (1.0 / 24.0) * PN * std::pow(std::cos(m_lat), 4) * Pt * (5.0 - std::pow(Pt, 2) + 9.0 * std::pow(Pnn, 2) + 4.0 * std::pow(Pnn, 4)) * std::pow(PDL, 4) -
            (1.0 / 720.0) * PN * std::pow(std::cos(m_lat), 6) * Pt * (-61.0 + 58.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 270.0 * std::pow(Pnn, 2) + 330.0 * std::pow(Pt, 2) * std::pow(Pnn, 2)) * std::pow(PDL, 6) -
            (1.0 / 40320.0) * PN * std::pow(std::cos(m_lat), 8) * Pt * (-1385.0 + 3111 * std::pow(Pt, 2) - 543 * std::pow(Pt, 4) + std::pow(Pt, 6)) * std::pow(PDL, 8)) * Pmo);
        
        loc_x = static_cast<double>((PN * std::cos(m_lat) * PDL -
            1.0 / 6.0 * PN * std::pow(std::cos(m_lat), 3) * (-1 + std::pow(Pt, 2) - std::pow(Pnn, 2)) * std::pow(PDL, 3) -
            1.0 / 120.0 * PN * std::pow(std::cos(m_lat), 5) * (-5.0 + 18.0 * std::pow(Pt, 2) - std::pow(Pt, 4) - 14.0 * std::pow(Pnn, 2) + 58.0 * std::pow(Pt, 2) *std::pow(Pnn, 2)) * std::pow(PDL, 5) -
            1.0 / 5040.0 * PN * std::pow(std::cos(m_lat), 7) * (-61.0 + 479.0 * std::pow(Pt, 2) - 179.0 * std::pow(Pt, 4) + std::pow(Pt, 6)) * std::pow(PDL, 7)) * Pmo + 250000);

        loc_z = ell;
    }

    double getloc_x()
    {
        return loc_x;
    }

    double getloc_y()
    {
        return loc_y;
    }

    double getloc_z()
    {
        return loc_z;
    }
} // namespace earth