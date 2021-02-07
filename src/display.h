#ifndef INCLUDE_AKFSFSIM_DISPLAY_H
#define INCLUDE_AKFSFSIM_DISPLAY_H

// System Includes
#include <memory>
#include <string>
#include <iostream>
#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

template<typename ... Args>
std::string string_format( const std::string& format, Args ... args )
{
    size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
    if( size <= 0 ){ throw std::runtime_error( "Error during formatting." ); }
    std::unique_ptr<char[]> buf( new char[ size ] ); 
    snprintf( buf.get(), size, format.c_str(), args ... );
    return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
}

struct Vector2
{
    double x,y;
    Vector2():x(0.0),y(0.0){};
    Vector2(double _x, double _y):x(_x),y(_y){};
};

std::vector<Vector2> transformPoints(const std::vector<Vector2>& points, const Vector2& position, const double rotation);
std::vector<std::vector<Vector2>> transformPoints(const std::vector<std::vector<Vector2>>& dataset, const Vector2& position, const double rotation);
std::vector<Vector2> offsetPoints(const std::vector<Vector2>& points, const Vector2& offset);
std::vector<std::vector<Vector2>> offsetPoints(const std::vector<std::vector<Vector2>>& dataset, const Vector2& offset);


class Display
{
    public:

        Display();
        ~Display();

        bool createRenderer( std::string title, int screenWidth, int screenHeight );
        void destroyRenderer(); 

        void showScreen();
        void clearScreen();

        double getScreenWidth()const {return mScreenWidth;}
        double getScreenHeight() const {return mScreenHeight;}
        double getScreenAspectRatio() const {return getScreenWidth()/getScreenHeight(); }

        void drawText_MainFont(const std::string text, const Vector2 pos, const double scale = 1, const SDL_Color color = {0,0,0}, bool centered = false );

        void setDrawColour(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha=0xFF);
        void setView(double width, double height, double xOffset, double yOffset);
        void setView(double xOffset, double yOffset);
        void drawLine(const Vector2& startPos, const Vector2& endPos);
        void drawLines(const std::vector<Vector2> &points);
        void drawLines(const std::vector<std::vector<Vector2>>& points);

    private:

        Vector2 transformPoint(const Vector2& point);

        int mScreenWidth;
        int mScreenHeight;

        double mViewWidth;
        double mViewHeight;
        double mViewXOffset;
        double mViewYOffset;

        SDL_Window* mWindow;
        SDL_Renderer* mRenderer;
        TTF_Font *mMainFont;
        
};

#endif //INCLUDE_AKFSFSIM_DISPLAY_H