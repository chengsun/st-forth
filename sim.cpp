#include <SDL/SDL.h>
#include <cstdio>
#include <climits>
#include <cmath>
#include <algorithm>
using namespace std;

#define PI 3.14159265359
#define DELAYCONSTANT 2

SDL_Surface *screen, *drawn;

int gtheta[3] = {0,0,0};
bool pen_state = false;
int debugplotx, debugploty;

typedef struct _Point {
    float x, y;
} Point;

void set_pixel(SDL_Surface *surface, int x, int y, uint32_t colour)
{
    if (x < 1 || x >= surface->w || y < 1 || y >= surface->h) return;
    switch (surface->format->BytesPerPixel) {
    case 4:
        for (int dy = -1; dy <= 0; ++dy)
        for (int dx = -1; dx <= 0; ++dx)
        ((Uint32 *)surface->pixels)[(y+dy)*surface->pitch/4 + x+dx] = colour;
        break;
    default:
        fprintf(stderr, "Can't set pixel: unknown format (%d Bpp). FIXME!\n", surface->format->BytesPerPixel);
        exit(1);
    }
}

void draw_line(SDL_Surface *surface, Point *a, Point *b, uint32_t colour)
{
    float x1 = a->x, x2 = b->x,
          y1 = a->y, y2 = b->y;

    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if(steep)
    {
        swap(x1, y1);
        swap(x2, y2);
    }

    if(x1 > x2)
    {
        swap(x1, x2);
        swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for(int x=(int)x1; x<maxX; x++)
    {
        if(steep)
        {
            set_pixel(surface, y,x, colour);
        }
        else
        {
            set_pixel(surface, x,y, colour);
        }

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}

void get_endpoints(int mtheta[3], Point points[3]) {
    Point pt = {300,300};
    float theta = 0;
    for (int i = 0; i < 3; ++i) {
        theta += mtheta[i];
        pt.x += cos(theta * PI/180.) * 100.;
        pt.y += sin(theta * PI/180.) * 100.;
        points[i] = pt;
    }
}

Point get_endpoint(int mtheta[3]) {
    Point points[3];
    get_endpoints(mtheta, points);
    return points[2];
}

void update() {
    if (SDL_BlitSurface(drawn, NULL, screen, NULL) < 0) {
        fprintf(stderr, "SDL blit failed (%s)", SDL_GetError());
        exit(1);
    }

    if (SDL_LockSurface(screen) < 0) {
        fprintf(stderr, "SDL lock screen failed (%s)", SDL_GetError());
        return;
    }
    Point points[4] = {{300,300}};
    get_endpoints(gtheta, points+1);
    for (int i = 0; i < 3; ++i) {
        draw_line(screen, &points[i], &points[i+1], SDL_MapRGB(drawn->format, 255, 0, 0));
    }
    set_pixel(screen, debugplotx, debugploty, SDL_MapRGB(drawn->format, 0, 0, 255));

    SDL_UnlockSurface(screen);
    SDL_UpdateRect(screen, 0, 0, 0, 0);
}

void motor_move(int motor, int mtheta) {
    mtheta -= 80;
    if (mtheta < -80) mtheta = -80;
    if (mtheta > 80) mtheta = 80;
    if (DELAYCONSTANT) {
        while (1) {
            int mdelta = mtheta - gtheta[motor];
            if (mdelta == 0) break;
            int mstep = mdelta < 0 ? -1 : +1;
            Point before = get_endpoint(gtheta);
            gtheta[motor] += mstep;
            Point after = get_endpoint(gtheta);
            if (pen_state) {
                if (SDL_LockSurface(drawn) < 0) {
                    fprintf(stderr, "SDL lock screen failed (%s)", SDL_GetError());
                } else {
                    draw_line(drawn, &before, &after, SDL_MapRGB(drawn->format, 0, 0, 0));
                    SDL_UnlockSurface(drawn);
                }
            }
            update();
            SDL_Delay(1*DELAYCONSTANT);
        }
    } else {
        gtheta[motor] = mtheta;
        update();
    }
    SDL_Delay(10*DELAYCONSTANT);
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        fprintf(stderr, "SDL init failed (%s)\n" , SDL_GetError());
        exit(1);
    }
    atexit(SDL_Quit);

    screen = SDL_SetVideoMode(600, 600, 32, SDL_SWSURFACE);
    if (!screen) {
        fprintf(stderr, "Screen create failed (%s)\n" , SDL_GetError());
        exit(1);
    }

    drawn = SDL_CreateRGBSurface(SDL_SWSURFACE, 600, 600, 32, 0xff0000, 0x00ff00, 0x0000ff, 0);
    if (!drawn) {
        fprintf(stderr, "Drawn create failed (%s)\n" , SDL_GetError());
        exit(1);
    }
    if (SDL_FillRect(drawn, NULL, SDL_MapRGB(drawn->format, 255, 255, 255)) < 0) {
        fprintf(stderr, "Drawn fillrect failed (%s)\n" , SDL_GetError());
        exit(1);
    }

    while (1) {
        update();

        int ch;
        while (isspace(ch = getchar()));
        switch (ch) {
        case 'M': {
            int motor, mtheta;
            if (scanf("%d %d", &motor, &mtheta) < 2) {
                fprintf(stderr, "Input stream: failed to parse move command\n");
                exit(1);
            }
            if (motor < 0 || motor > 2) {
                fprintf(stderr, "Input stream: invalid motor number %d", motor);
                exit(1);
            }
            fprintf(stderr, "Motor %d to %d\n", motor, mtheta);
            motor_move(motor, mtheta);
            putchar('\n');
            fflush(stdout);
            break;
        }
        case 'P': {
            int pen;
            if (scanf("%d", &pen) < 1) {
                fprintf(stderr, "Input stream: failed to parse pen command\n");
                exit(1);
            }
            if (pen < 0 || pen > 1) {
                fprintf(stderr, "Input stream: invalid pen parameter %d\n", pen);
                exit(1);
            }
            pen_state = (pen==1);
            if (pen_state) {
                Point pt = get_endpoint(gtheta);
                set_pixel(drawn, pt.x, pt.y, SDL_MapRGB(drawn->format, 0, 0, 0));
            }
            fprintf(stderr, "Pen state to %d\n", pen_state);
            SDL_Delay(10*DELAYCONSTANT);
            putchar('\n');
            fflush(stdout);
            break;
        }
        case '*': {
            while (getchar() != '\n');
            break;
        }
        case '+': {
            if (scanf("%d %d", &debugplotx, &debugploty) < 2) {
                fprintf(stderr, "Input stream: failed to parse debugplot command\n");
                exit(1);
            }
            debugplotx = debugplotx / 100 + 300;
            debugploty = debugploty / 100 + 300;
            break;
        }
        case EOF:
            fprintf(stderr, "Input stream: finished\n");
            goto quitloop;
        default:
            fprintf(stderr, "Input stream: unknown command '%c'\n", ch);
            exit(1);
        }
    }

    fprintf(stderr, "Waiting\n");

quitloop: {
        SDL_Event evt;
        while (1) {
            SDL_WaitEvent(&evt);
            if (evt.type == SDL_QUIT) break;
        }
    }
    
    return 0;
}
