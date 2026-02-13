#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define MAX_RANGES 10000

int main()
{

    // 1- URL ile  dosya indiriliyor.

    char url[400];
    printf("Lidar dosya linkini giriniz(URL):\n");
    scanf("%s", url);

    const char *filename = "lidar_data.toml";

    char command[400];
    sprintf(command, "curl -s -L -o %s %s", filename, url);

    int result = system(command);

    if (result != 0)
    {
        printf("Dosya indirilemedi!\n");
        return 1;
    }

    // 2-İndirilen dosya açılıyor.

    FILE *fp = fopen(filename, "r");

    if (fp == NULL)
    {
        printf("Dosya acilamadi. Baglantinizi konrtol ediniz!\n");
        return 1;
    }

    // 3-İndirilen dosyayı satır satır okuyup gerekli verileri alıyor ve yazdırıyor.

    double angle_min = 0.0, angle_max = 0.0, angle_increment = 0.0, range_min = 0.0, range_max = 0.0;

    char line[800];

    double ranges[MAX_RANGES];

    int range_count = 0;

    int in_ranges_control = 0;

    while (fgets(line, sizeof(line), fp))
    {
        if (strstr(line, "angle_min"))
            sscanf(line, "angle_min = %lf", &angle_min);
        else if (strstr(line, "angle_max"))
            sscanf(line, "angle_max = %lf", &angle_max);
        else if (strstr(line, "angle_increment"))
            sscanf(line, "angle_increment = %lf", &angle_increment);
        else if (strstr(line, "range_min"))
            sscanf(line, "range_min = %lf", &range_min);
        else if (strstr(line, "range_max"))
            sscanf(line, "range_max = %lf", &range_max);
        else if (strstr(line, "ranges = ["))
        {
            in_ranges_control = 1;
        }

        if (in_ranges_control)
        {
            if (strchr(line, ']'))
                in_ranges_control = 0;

            char *p = line;

            while (*p)
            {
                double value;
                if (sscanf(p, "%lf", &value) == 1)
                {
                    ranges[range_count] = value;
                    range_count++;
                }

                p = strchr(p, ',');

                if (p)
                    p++;

                else
                    break;
            }
        }
    }

    fclose(fp);

    printf("\n\nTOML dosyasi okundu.\n\n");
    printf("angle_min=%.3f,angle_max=%.3f,angle_inc=%.6f\n\n", angle_min, angle_max, angle_increment);
    printf("range_min=%.2f,range_max=%.2f\n\n", range_min, range_max);
    printf("Toplam okunan range sayisi: %d\n\n", range_count);

    // Dosyadan alınan verileri istenen koşullara göre filtreliyor.

    double filtered_range[MAX_RANGES];

    int filtered_range_index[MAX_RANGES];

    int filtered_range_count = 0;

    for (int i = 0; i < range_count; i++)
    {
        double r = ranges[i];

        if (r < range_max && r > range_min && r != -1 && r != -999.0 && r != 999.0)
        {
            filtered_range[filtered_range_count] = r;
            filtered_range_index[filtered_range_count] = i;
            filtered_range_count++;
        }
    }

    printf("Filtrelenmis range sayisi : %d\n\n", filtered_range_count);

    double x[MAX_RANGES], y[MAX_RANGES];

    int pointCount = 0;

    // Filtrelenen verilere Polar-Kartezyen dönüşümü yapıyor.

    for (int i = 0; i < filtered_range_count; i++)
    {
        double r = filtered_range[i];

        int index = filtered_range_index[i];

        double angle = angle_min + index * angle_increment;

        x[pointCount] = r * cos(angle);
        y[pointCount] = r * sin(angle);
        pointCount++;
    }

    printf("Kartezyen donusumu basariyla tamamlandi. Nokta sayisi: %d\n\n", pointCount);

    // RANSAC algoritması başlıyor.

    srand(time(NULL));

    double a[800], b[800], c[800];

    int inlierCount[800];

    int lineCount = 0;

    int used[MAX_RANGES] = {0};

    double max_point_dist = 0.012;
    int minPoints = 8;

    // Rastgele kullanılmamış iki nokta seçiyor ve bunlardan doğru denklemleri elde ediyor.
    for (int iter = 0; iter < 1500; iter++)
    {
        int i1 = rand() % pointCount;
        int i2 = rand() % pointCount;

        if (i1 == i2 || used[i1] || used[i2])
            continue;

        double x1 = x[i1], y1 = y[i1];
        double x2 = x[i2], y2 = y[i2];

        double A = y2 - y1;
        double B = x1 - x2;
        double C = x2 * y1 - x1 * y2;

        // Katsayıları birim vektöre çeviriyor.

        double norm = sqrt(A * A + B * B);
        A /= norm;
        B /= norm;
        C /= norm;

        // Noktaları kontrol ediyor max_point_dist mesafesinden kısa olan mesafedeki noktaları doğruya ekliyor.

        int inliers = 0;

        for (int i = 0; i < pointCount; i++)
        {
            if (used[i])
                continue;

            double dist = fabs((A * x[i]) + (B * y[i]) + C);

            if (dist < max_point_dist)
            {
                inliers++;
            }
        }

        if (inliers >= minPoints)
        {

            // Doğru kriterleri karşılıyorsa doğruyu kaydet.

            a[lineCount] = A;
            b[lineCount] = B;
            c[lineCount] = C;
            inlierCount[lineCount] = inliers;

            // Kullanılmış noktaları tekrar kullanılmaması için kaydediyor.

            for (int i = 0; i < pointCount; i++)
            {
                if (used[i])
                    continue;

                double dist = fabs((A * x[i]) + (B * y[i]) + C);

                if (dist < max_point_dist)
                    used[i] = 1;
            }

            printf("Yeni Dogru %d bulundu --> A=%.6f, B=%.6f, C=%.6f, Inliers=%d\n", lineCount + 1, A, B, C, inliers);
            lineCount++;
        }
    }

    printf("\nToplam Bulunan Dogru Sayisi: %d\n\n", lineCount);

    printf("\nDogrularin kartezyen koordinatlari:\n\n");
    
    for (int i = 0; i < lineCount; i++)
    {
        printf("Dogru %d: %.6fx + %.6fy + %.6f = 0\n",
               i + 1, a[i], b[i], c[i]);
    }

    // Doğruların kesişimleri bulunuyor.Kesişimi olmayan(det=0) olan doğrular atlanıyor.

    double interX[1000], interY[1000];

    int interCount = 0;

    for (int i = 0; i < lineCount; i++)
    {
        for (int j = i + 1; j < lineCount; j++)
        {
            double A1 = a[i], B1 = b[i], C1 = c[i];
            double A2 = a[j], B2 = b[j], C2 = c[j];

            double det = A1 * B2 - A2 * B1;
            if (det == 0)
                continue;

            double x_int = (B1 * C2 - B2 * C1) / det;
            double y_int = (C1 * A2 - C2 * A1) / det;

            interX[interCount] = x_int;
            interY[interCount] = y_int;

            interCount++;
        }
    }

    printf("\nToplam %d adet kesisim noktasi bulundu.\n", interCount);

    printf("\nKesisim noktalarinin kartezyen koordinatlari ve robota uzakliklari:\n\n");
    

    // Her kesişim noktasının robota(0,0) uzaklığı hesaplanıyor.

    double dist[1000];
    for (int i = 0; i < interCount; i++)
    {
        dist[i] = sqrt(interX[i] * interX[i] + interY[i] * interY[i]);

        printf("Nokta %d: (%.3f, %.3f) --> Noktanin robota uzakligi = %.3f m\n",
               i + 1, interX[i], interY[i], dist[i]);
    }

    printf("\n\n");
    

    // Küçük açısı 60 dereceden büyük olan kesişim noktalarını buluyor.

    double PI_number = 3.14159265358979323846;

    double interX60[1000], interY60[1000];
    int interCount60 = 0;

    printf("Aralarindaki aci 60 dereceden buyuk olan dogru ciftleri:\n\n");

    for (int i = 0; i < lineCount; i++)
    {
        double slope1 = -a[i] / b[i];
        double angle1 = atan(slope1);

        for (int j = i + 1; j < lineCount; j++)
        {
            double slope2 = -a[j] / b[j];
            double angle2 = atan(slope2);

            double angle_diff = fabs(angle1 - angle2);
            double angle_deg = angle_diff * 180.0 / PI_number;

            if (angle_deg > 90.0)
                angle_deg = 180.0 - angle_deg;

            if (angle_deg > 60.0)
            {
                double A1 = a[i], B1 = b[i], C1 = c[i];
                double A2 = a[j], B2 = b[j], C2 = c[j];

                double det = A1 * B2 - A2 * B1;
                if (det == 0)
                    continue;

                double x_int = (B1 * C2 - B2 * C1) / det;
                double y_int = (C1 * A2 - C2 * A1) / det;

                interX60[interCount60] = x_int;
                interY60[interCount60] = y_int;

                interCount60++;

                printf("Dogru %d ile Dogru %d arasindaki aci: %.2f derece (Kesisim Noktasi: %.2f, %.2f)\n",
                       i + 1, j + 1, angle_deg, x_int, y_int);
            }
        }
    }
    printf("\nToplam %d adet 60° üzeri kesisim bulundu.\n", interCount60);

    // Robota(0,0) en yakın kesişim noktasını buluyor.

    double nearestX = 0.0, nearestY = 0.0;

    double nearestDist = 9999.0;

    int nearestIndex = -1;

    if (interCount > 0)
    {
        nearestX = interX[0];
        nearestY = interY[0];
        nearestDist = dist[0];
        nearestIndex = 0;

        for (int i = 1; i < interCount; i++)
        {
            if (dist[i] < nearestDist)
            {
                nearestDist = dist[i];
                nearestX = interX[i];
                nearestY = interY[i];
                nearestIndex = i;
            }
        }

        printf("\nEn yakin kesisim noktasi: (%.3f, %.3f) -> Uzaklik = %.3f m\n\n\n",
               nearestX, nearestY, nearestDist);
    }

    else
    {
        printf("\nKesisim noktasi bulunamadi, dolayisiyla ok cizilmeyecek.\n");
    }

    // Gnuplot dosyası açılıyor.

    FILE *gnuplotFile = fopen("cizim.gp", "w");

    if (gnuplotFile == NULL)
    {
        printf("Gnuplot script dosyasi olusturulamadi!\n");
        return 1;
    }

    // Gnuplot ayarları yapılıyor.

    fprintf(gnuplotFile, "set terminal pngcairo size 1024,768 enhanced font 'Verdana,10'\n");
    fprintf(gnuplotFile, "set output 'proje_ciktisi.png'\n");
    fprintf(gnuplotFile, "set title 'LIDAR Verisi - RANSAC Dogrular ve Kesisimler'\n");
    fprintf(gnuplotFile, "set xlabel 'X (metre)'\n");
    fprintf(gnuplotFile, "set ylabel 'Y (metre)'\n");
    fprintf(gnuplotFile, "set grid\n");
    fprintf(gnuplotFile, "set key outside right top\n");
    fprintf(gnuplotFile, "set size square\n");
    fprintf(gnuplotFile, "set xrange [-4:4]\n");
    fprintf(gnuplotFile, "set yrange [-4:4]\n");

    fprintf(gnuplotFile, "set object 1 circle at 0,0 size 0.05 fillcolor rgb 'red' fs solid border rgb 'black' lw 2 front\n");

    // En yakın kesişim noktasına kırmızı kesikli çizgi çekiliyor ve uzaklık metre cinsinden yazılıyor.

    if (nearestIndex != -1)
    {
        fprintf(gnuplotFile, "set arrow 1 from 0,0 to %f,%f dashtype 2 linecolor rgb 'red' lw 3 head filled size 0.1,15,45 front\n", nearestX, nearestY);

        double label_x = nearestX / 3.0;
        double label_y = nearestY / 3.0;
        fprintf(gnuplotFile, "set object 2 rectangle at %f,%f size 0.5,0.25 fillcolor rgb 'yellow' fs solid border rgb 'red' front\n", label_x, label_y);
        fprintf(gnuplotFile, "set label 1 '%.2f m' at %f,%f textcolor rgb 'black' font ',11,bold' center front\n", nearestDist, label_x, label_y);
    }

    // Gnuplot için gerekli verileri içerecek dosyalar açılıyor.

    FILE *fp_points = fopen("noktalar.dat", "w");
    FILE *fp_lines = fopen("dogrular.dat", "w");
    FILE *fp_inliers = fopen("inlier.dat", "w");
    FILE *fp_intersections = fopen("kesisimler.dat", "w");
    FILE *fp_intersections60 = fopen("kesisim60.dat", "w");

    if (!fp_points || !fp_lines || !fp_inliers || !fp_intersections || !fp_intersections60)
    {
        printf("Veri dosyalarindan biri olusturulamadi!\n");
        return 1;
    }

    // Burdan itibaren sırayla dosyalara veriler işleniyor.

    for (int i = 0; i < pointCount; i++)
    {
        fprintf(fp_points, "%f %f\n", x[i], y[i]);
        if (used[i])
            fprintf(fp_inliers, "%f %f\n", x[i], y[i]);
    }

    fclose(fp_points);
    fclose(fp_inliers);

    double xmin = -3.5, xmax = 3.5;

    for (int i = 0; i < lineCount; i++)
    {
        double A = a[i], B = b[i], C = c[i];

        // Doğrunun dik olduğu özel durumlar için kontrol ediliyor.

        if (fabs(B) < 0.1)
        {
            double x_line = -C / A;
            fprintf(fp_lines, "%f %f\n%f %f\n\n", x_line, -3.5, x_line, 3.5);
        }

        else
        {
            double y1 = (-A * xmin - C) / B;
            double y2 = (-A * xmax - C) / B;
            fprintf(fp_lines, "%f %f\n%f %f\n\n", xmin, y1, xmax, y2);
        }
    }

    fclose(fp_lines);

    for (int i = 0; i < interCount; i++)
        fprintf(fp_intersections, "%f %f\n", interX[i], interY[i]);
    fclose(fp_intersections);

    for (int i = 0; i < interCount60; i++)
        fprintf(fp_intersections60, "%f %f\n", interX60[i], interY60[i]);
    fclose(fp_intersections60);

    // Doğruların ve noktaların nasıl görüneceği ayarlanıyor.

    fprintf(gnuplotFile,
            "plot 'noktalar.dat' with points pointtype 7 pointsize 0.3 lc rgb 'gray' title 'Outlier Noktalar',\\\n"
            "     'inlier.dat' with points pointtype 7 pointsize 0.5 lc rgb 'green' title 'Inlier Noktalar',\\\n"
            "     'dogrular.dat' with lines lc rgb 'dark-green' lw 1.5 title 'RANSAC Dogrular',\\\n"
            "     'kesisimler.dat' with points pointtype 7 pointsize 0.7 lc rgb 'violet' title 'Kesisim Noktalari',\\\n"
            "     'kesisim60.dat' with points pointtype 1 pointsize 2 lc rgb 'yellow' title 'Aci > 60° Kesisimler'\n");

    fclose(gnuplotFile);

    system("gnuplot cizim.gp");

    printf("Gorsel 'proje_ciktisi.png' olarak kaydedildi.\n\n");

    printf("Program sonlaniyor...\n\n");
    return 0;
} 