#include <spot_comm/ImageServiceImpl.h>


  Status ImageServiceImpl::ListImageSources(ServerContext* context, const ListImageSourcesRequest* request,
                  ListImageSourcesResponse* response) {
    //create response to list all the possible image sources
    ImageSource* imgsrc = response->add_image_sources();
    imgsrc->set_name("imageName");
    imgsrc->set_rows(480);
    imgsrc->set_cols(640);
    imgsrc->set_depth_scale(1000);
    imgsrc->set_image_type(ImageSource::IMAGE_TYPE_VISUAL);

    ImageSource* imgsrc2 = response->add_image_sources();
    imgsrc2->set_name("imageName2");
    imgsrc2->set_rows(640);
    imgsrc2->set_cols(480);
    imgsrc2->set_depth_scale(1000);
    imgsrc2->set_image_type(ImageSource::IMAGE_TYPE_VISUAL);

    for(int i = 0; i < response->image_sources_size(); i++) {
      std::cout << "Name: " << response->image_sources(i).name() << std::endl;
      std::cout << "Rows: " << response->image_sources(i).rows() << std::endl;
      std::cout << "Cols: " << response->image_sources(i).cols() << std::endl;
      std::cout << "Depth scale: " << response->image_sources(i).depth_scale() << std::endl;
      std::cout << "Type: " << response->image_sources(i).image_type() << std::endl;
      std::cout << " " << std::endl;
    }
    
    return Status::OK;
  }

  Status ImageServiceImpl::GetImage(ServerContext* context, const GetImageRequest* request,
                  GetImageResponse* response) {

      for(int i = 0; i < request->image_requests_size(); i++) {
        ImageResponse* img = response->add_image_responses();
        ImageCapture* shot;
        ImageSource* imgsrc;
        img->set_allocated_shot(shot);

        img->set_allocated_source(imgsrc);
        img->mutable_source()->set_name(request->image_requests(i).image_source_name());
        img->mutable_source()->set_rows(480 - i * 10);
        img->mutable_source()->set_cols(640 - i * 10);
        img->mutable_source()->set_depth_scale(1000);
        img->mutable_source()->set_image_type(ImageSource::IMAGE_TYPE_VISUAL);

        img->set_status(ImageResponse::STATUS_OK);
      }

      return Status::OK;
      
  }
