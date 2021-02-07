// System Includes
#include <string>
#include <iostream>
#include <vector>

// Program Includes
#include "display.h"

Display::Display()
:mScreenWidth(0),
 mScreenHeight(0),
 mWindow(nullptr),
 mRenderer(nullptr),
 mMainFont(nullptr)
{}

Display::~Display()
{
    destroyRenderer();
}

bool Display::createRenderer( std::string title, int screenWidth, int screenHeight )
{
    destroyRenderer();

    mScreenWidth = screenWidth;
    mScreenHeight = screenHeight;

    mViewWidth = mScreenWidth;
    mViewHeight = mScreenHeight;
    mViewXOffset = 0.0;
    mViewYOffset = 0.0;
    
    // Create window
    mWindow = SDL_CreateWindow( title.c_str(), SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, screenWidth, screenHeight, 0 );
    if( mWindow == nullptr )
    {
        std::cout << "Window could not be created! SDL_Error: " << SDL_GetError() << std::endl;
        destroyRenderer();
        return false;
    }

    // Create Renderer
    mRenderer = SDL_CreateRenderer( mWindow, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC );
    if( mRenderer == nullptr )
    {
        std::cout << "Renderer could not be created! SDL Error: " << SDL_GetError() << std::endl;
        destroyRenderer();
        return false;
    }

    // Create Font
    mMainFont = TTF_OpenFont( "Roboto-Regular.ttf", 18 );
    if( mMainFont == nullptr )
    {
        std::cout << "Failed to load font! SDL_ttf Error: " <<  TTF_GetError() << std::endl;
        destroyRenderer();
        return false;
    }

    return true;
}

void Display::destroyRenderer()
{
    // Destroy Renderer
    if (mRenderer != nullptr)
    {
        SDL_DestroyRenderer( mRenderer );
        mRenderer = nullptr;
    }

    // Destroy Window
    if(mWindow != nullptr)
    {
        SDL_DestroyWindow(mWindow);
        mWindow = nullptr;
    }

    // Destroy Font
    if(mMainFont != nullptr)
    {
        TTF_CloseFont(mMainFont);
        mMainFont = nullptr;
    }
}

void Display::clearScreen()
{
    if (mRenderer != nullptr)
    {
        //Clear screen
        SDL_SetRenderDrawColor( mRenderer, 0x00, 0x00, 0x00, 0xFF );
        SDL_RenderClear( mRenderer );
    }
}

void Display::showScreen()
{
    if (mRenderer != nullptr)
    {
        SDL_RenderPresent( mRenderer );
    }
}

void Display::setView(double width, double height, double xOffset, double yOffset)
{
    mViewWidth = fabs(width);
    mViewHeight = fabs(height);
    mViewXOffset = xOffset - mViewHeight/2.0;
    mViewYOffset = yOffset - mViewWidth/2.0;
}

void Display::setView(double xOffset, double yOffset)
{
    mViewXOffset = xOffset - mViewHeight/2.0;
    mViewYOffset = yOffset - mViewWidth/2.0;
}

void Display::setDrawColour(uint8_t red, uint8_t green, uint8_t blue, uint8_t alpha){SDL_SetRenderDrawColor( mRenderer, red, green, blue, alpha );}

void Display::drawLine(const Vector2& startPos, const Vector2& endPos)
{
    Vector2 p1 = transformPoint(startPos);
    Vector2 p2 = transformPoint(endPos);
    SDL_RenderDrawLine( mRenderer, p1.x, p1.y, p2.x, p2.y );
}

void Display::drawLines(const std::vector<Vector2> &points){for (unsigned int i=1; i < points.size(); ++i){drawLine(points[i-1],points[i]);}}
void Display::drawLines(const std::vector<std::vector<Vector2>>& dataset){for (const std::vector<Vector2>& points : dataset){drawLines(points);}}

Vector2 Display::transformPoint(const Vector2& point)
{
    double dx = point.x - mViewXOffset;
    double dy = point.y - mViewYOffset;
    double y = mScreenHeight - (dx/mViewHeight)*mScreenHeight;
    double x = (dy/mViewWidth)*mScreenWidth;
    return Vector2(x,y);
}

void Display::drawText_MainFont(const std::string text,const Vector2 pos, const double scale, const SDL_Color color, bool centered)
{
    SDL_Surface* surface = TTF_RenderText_Blended(mMainFont,text.c_str(), color);
    if (surface == nullptr)
    {
        std::cout <<  "Unable to render text surface! SDL_ttf Error: " << TTF_GetError() << std::endl;
        return;
    }
    
    SDL_Texture* texture = SDL_CreateTextureFromSurface( mRenderer, surface );
    if (texture == nullptr)
    {
        SDL_FreeSurface(surface);
        std::cout << "Unable to create texture from rendered text! SDL Error: " << SDL_GetError() << std::endl;
        return;
    }

    int width = surface->w * scale;
    int height = surface->h * scale;
    int x = pos.x;
    int y = pos.y;

    if(centered)
    {
        x -= width/2;
        y -= height/2;
    }

    SDL_Rect renderQuad = { x, y, width, height };
    SDL_RenderCopy(mRenderer, texture, nullptr, &renderQuad);

    SDL_DestroyTexture(texture);
    SDL_FreeSurface(surface);
}

std::vector<Vector2> transformPoints(const std::vector<Vector2>& points, const Vector2& position, const double rotation)
{
    double ctheta = cos(rotation);
    double stheta = sin(rotation);
    std::vector<Vector2> transformedPoints;
    for (const Vector2& point : points)
    {
        double x = point.x * ctheta - stheta * point.y + position.x;
        double y = point.x * stheta + ctheta * point.y + position.y;
        transformedPoints.push_back(Vector2(x,y));
    }
    return transformedPoints;
}
std::vector<std::vector<Vector2>> transformPoints(const std::vector<std::vector<Vector2>>& dataset, const Vector2& position, const double rotation)
{
    std::vector<std::vector<Vector2>> transformedDataset;
    for (const std::vector<Vector2> points : dataset){transformedDataset.push_back(transformPoints(points, position, rotation));}
    return transformedDataset;
}

std::vector<Vector2> offsetPoints(const std::vector<Vector2>& points, const Vector2& offset)
{
    std::vector<Vector2> transformedPoints;
    for (const Vector2& point : points){transformedPoints.push_back(Vector2(point.x + offset.x,point.y + offset.y));}
    return transformedPoints;
}

std::vector<std::vector<Vector2>> offsetPoints(const std::vector<std::vector<Vector2>>& dataset, const Vector2& offset)
{
    std::vector<std::vector<Vector2>> transformedDataset;
    for (const std::vector<Vector2> points : dataset){transformedDataset.push_back(offsetPoints(points, offset));}
    return transformedDataset;
}